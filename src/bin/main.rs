#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![feature(addr_parse_ascii)]
#[derive(Debug)]
pub enum MqttError {
    InvalidState,
    InvalidPacket,
    ConnectionRefused,
    Disconnected,
}
#[derive(Debug)]
pub enum SensorError {
    ChecksumMismatch,
    Timeout,
    PinError,
}

use core::{
    fmt::Debug,
    net::{IpAddr, Ipv4Addr},
};

use alloc::format;
use bt_hci::{cmd::info, controller::ExternalController, event::le, uuid::appearance::sensor};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_net::{
    tcp::{self, TcpSocket},
    Config, DhcpConfig, Runner, Stack, StackResources,
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::{Delay, Duration, Timer};
use embedded_io_async::Write;
use esp32_dht11_rs::{Reading, DHT11};
use esp_hal::{
    analog::adc::{Adc, AdcConfig, AdcPin}, clock::{self, CpuClock}, gpio::{AnyPin, Level, Output, OutputConfig, Pin}, peripherals::{ADC1, GPIO4}, timer::{systimer::SystemTimer, timg::TimerGroup}, Async, DriverMode
};
use esp_wifi::{
    ble::controller::BleConnector,
    wifi::{
        ClientConfiguration, Configuration, Interfaces, WifiController, WifiDevice, WifiEvent,
        WifiState,
    },
    EspWifiController,
};
use tinymqtt::{self, MqttClient};
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const HOSTNAME: &str = env!("HOSTNAME");
const MQTT_CLIENT_ID: &str = env!("MQTT_CLIENT_ID");
const MQTT_BROKER: &str = env!("MQTT_BROKER");
const MQTT_PORT: &str = env!("MQTT_PORT");
const MQTT_TOPIC: &str = env!("MQTT_TOPIC");

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

type DHTSensorChannel = Channel<NoopRawMutex, Reading, 1>;
type LuxSensorChannel = Channel<NoopRawMutex, u16, 1>;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.4.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init = &*mk_static!(
        EspWifiController<'static>,
        esp_wifi::init(timer1.timer0, rng, peripherals.RADIO_CLK)
            .expect("Failed to initialize WIFI/BLE controller")
    );
    let (mut wifi_controller, wifi_interfaces) = esp_wifi::wifi::new(&wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller");

    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport = BleConnector::new(&wifi_init, peripherals.BT);
    let _ble_controller = ExternalController::<_, 20>::new(transport);

    let wifi_device: WifiDevice<'_> = wifi_interfaces.sta;
    let mut dhcp_config = DhcpConfig::default();
    dhcp_config.hostname =
        Some(heapless::String::try_from(HOSTNAME).expect("Hostname is too long"));
    let wifi_config = Config::dhcpv4(dhcp_config);
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let (sta_stack, sta_runner) = embassy_net::new(
        wifi_device,
        wifi_config,
        mk_static!(StackResources<4>, StackResources::<4>::new()),
        seed,
    );

    // TODO: Spawn some tasks

    spawner.spawn(connection(wifi_controller)).ok();
    spawner.spawn(net_task(sta_runner)).ok();
    // spawner.spawn(counter()).ok();
    info!("Run in background");
    info!("Starting MQTT task");
    while !sta_stack.is_link_up() {
        info!("=> Connecting...");
        Timer::after(Duration::from_millis(500)).await;
    }
    Timer::after(Duration::from_millis(500)).await;
    // Blinky LED every 500ms
    let mut led = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());
    spawner.spawn(toggle(led)).ok();
    // DHT11 task
    let dht_sensor_channel = &*mk_static!(DHTSensorChannel, Channel::new());
    let mut dht_pin = peripherals.GPIO3.degrade();
    spawner.spawn(dht_task(dht_pin, dht_sensor_channel)).ok();
    // lux sensor task
    let lux_sensor_channel = &*mk_static!(LuxSensorChannel, Channel::new());
    let mut adc1_config = AdcConfig::new();
    let pin = adc1_config.enable_pin(peripherals.GPIO4, esp_hal::analog::adc::Attenuation::_0dB);
    let adc_1 = Adc::new(peripherals.ADC1, adc1_config).into_async();
    spawner.spawn(lux_task(pin, adc_1, lux_sensor_channel)).ok();
    // MQTT task
    spawner.spawn(mqtt_task(sta_stack, dht_sensor_channel, lux_sensor_channel)).ok();
    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.1/examples/src/bin
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("Wifi starting...");
    loop {
        let client_config = Configuration::Client(ClientConfiguration {
            ssid: SSID.into(),
            password: PASSWORD.into(),
            auth_method: esp_wifi::wifi::AuthMethod::WPAWPA2Personal,
            ..Default::default()
        });
        match controller.set_configuration(&client_config) {
            Ok(_) => info!("Set_configuration successful"),
            Err(e) => warn!("Set_configuration unsuccessful: {:?}", e),
        };
        match controller.set_mode(esp_wifi::wifi::WifiMode::Sta) {
            Ok(_) => info!("Set_wifi_mode successful"),
            Err(e) => warn!("Set_wifi_mode unsuccessful: {:?}", e),
        }
        match controller.start_async().await {
            Ok(_) => info!("Start_wifi successful"),
            Err(e) => warn!("Start_wifi unsuccessful: {:?}", e),
        }

        match controller.connect_async().await {
            Ok(_) => {
                info!("Connection successful");
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                warn!("Disconnected...");
            }
            Err(e) => {
                warn!("Connection unsuccessful: {:?}", e);
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await;
}

#[embassy_executor::task]
async fn counter() {
    let mut count = 0;
    loop {
        info!("{}", count);
        count += 1;
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn toggle(mut led: Output<'static>) {
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn mqtt_task(stack: Stack<'static>, dht_sensor_channel: &'static DHTSensorChannel, lux_sensor_channel: &'static LuxSensorChannel) {
    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 1024];
    let mut socket = tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    let remote_endpoint = (
        MQTT_BROKER
            .parse::<Ipv4Addr>()
            .expect("MQTT_BROKER is not valid"),
        MQTT_PORT.parse().expect("MQTT_PORT is not valid"),
    );
    while socket.connect(remote_endpoint).await.is_err() {
        warn!("Failed to connect to MQTT broker, retrying...");
        Timer::after(Duration::from_secs(1)).await;
    }
    socket.set_keep_alive(Some(Duration::from_secs(5)));
    while !socket.may_send() {
        info!("Socket state: {}", socket.state());
        Timer::after(Duration::from_millis(100)).await;
    }
    let mut mqtt_client: MqttClient<1024> = MqttClient::new();
    match socket
        .write_all(mqtt_client.connect(MQTT_CLIENT_ID, None).unwrap())
        .await
    {
        Ok(_) => {
            socket.flush().await.unwrap();
            while !socket.can_recv() {
                info!("Waiting for MQTT connection response...  ");
                Timer::after(Duration::from_millis(10)).await;
            }
            info!("Received MQTT connection response");
            let mut response_buf = [0u8; 1024];
            match socket.read(&mut response_buf).await {
                Ok(size) => {
                    info!("Read {} bytes from socket", size);
                    let response_slice = &response_buf[..size];
                    match mqtt_client
                        .receive_packet(response_slice, |mqtt_client, _topic, payload: &[u8]| {})
                    {
                        Ok(_) => info!("MQTT client connected successfully"),
                        Err(e) => warn!(
                            "Failed to connect MQTT client: {:?}",
                            format!("{:?}", e).as_str()
                        ),
                    }
                }
                Err(e) => {
                    warn!("Failed to read from socket: {:?}", e);
                    return;
                }
            };
        }
        Err(e) => warn!("Err: {:?}", e),
    }
    match socket.flush().await {
        Ok(_) => info!("Flushed---"),
        Err(e) => warn!("Err: {:?}", e),
    };
    while !mqtt_client.is_connected() {
        info!("---> {}", socket.state());
        // info!("MQTT client not connected, waiting...");
        mqtt_client.connect(MQTT_CLIENT_ID, None).unwrap();
        Timer::after(Duration::from_millis(100)).await;
    }
    loop {
        socket.wait_write_ready().await;
        let dht_data = match dht_sensor_channel.try_receive() {
            Ok(data) => {
                let (temperature, humidity) = (data.temperature, data.humidity);
                (temperature, humidity)
            }
            Err(_) => {
                let (temperature, humidity) = (0i8, 0u8);
                warn!("DHT11 sensor data not available, using default values");
                (temperature, humidity)
            }
        };
        let lux_data = match lux_sensor_channel.try_receive() {
            Ok(value) => value,
            Err(_) => {
                warn!("lux sensor data not available, using default value");
                0u16
            }
        };
        let payload = format!("{{\"sensor_id\": \"{}\", \"temperature\": {}, \"humidity\": {}, \"lux\": {}}}", MQTT_CLIENT_ID, dht_data.0, dht_data.1, lux_data);
        match mqtt_client.publish(MQTT_TOPIC, payload.as_bytes()) {
            Ok(packet) => match socket.write(packet).await {
                Ok(size) => {
                    info!("Publish success with {} bytes", size);
                    socket.flush().await.unwrap();
                }
                Err(e) => warn!("Err: {:?}", e),
            },
            Err(e) => {
                let err = format!("{:?}", e);
                warn!("Publishing error: {}", err.as_str())
            }
        }
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn dht_task(pin: AnyPin<'static>, dht_sensor_channel: &'static DHTSensorChannel) {
    let mut dht = DHT11::new(pin);
    loop {
        critical_section::with(|_| match dht.read() {
            Ok(data) => {
                info!("DHT11 read: OK, Temperature: {}, Humidity: {}", data.temperature, data.humidity);
                dht_sensor_channel.try_send(data).ok();
            }
            Err(error) => {
                let err = format!("{:?}", error);
                warn!("DHT11 read error: {}", err.as_str());
            }
        });
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn lux_task(mut pin: AdcPin<GPIO4<'static>, ADC1<'static>>, mut adc: Adc<'static, ADC1<'static>, Async>, lux_sensor_channel: &'static LuxSensorChannel) {
    loop {
        let value = adc.read_oneshot(&mut pin).await;
        info!("Light sensor read: OK {}", value);
        lux_sensor_channel.try_send(value).ok();
        Timer::after(Duration::from_secs(1)).await;
    }
}