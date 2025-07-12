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
use defmt::info;
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
    clock::{self, CpuClock},
    gpio::{AnyPin, Flex, Input, InputConfig, Io, Level, Output, OutputConfig, Pin},
    peripherals::{GPIO, GPIO3, I2C0},
    timer::{systimer::SystemTimer, timg::TimerGroup},
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

type SensorChannel = Channel<NoopRawMutex, Reading, 1>;

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

    let sensor_channel = &*mk_static!(SensorChannel, Channel::new());

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
    let mut dht_pin = peripherals.GPIO10.degrade();
    spawner.spawn(dht_task(dht_pin, sensor_channel)).ok();
    // MQTT task
    spawner.spawn(mqtt_task(sta_stack, sensor_channel)).ok();
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
            Err(e) => info!("Set_configuration unsuccessful: {:?}", e),
        };
        match controller.set_mode(esp_wifi::wifi::WifiMode::Sta) {
            Ok(_) => info!("Set_wifi_mode successful"),
            Err(e) => info!("Set_wifi_mode unsuccessful: {:?}", e),
        }
        match controller.start_async().await {
            Ok(_) => info!("Start_wifi successful"),
            Err(e) => info!("Start_wifi unsuccessful: {:?}", e),
        }

        match controller.connect_async().await {
            Ok(_) => {
                info!("Connection successful");
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                info!("Disconnected...");
            }
            Err(e) => {
                info!("Connection unsuccessful: {:?}", e);
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
async fn mqtt_task(stack: Stack<'static>, sensor_channel: &'static SensorChannel) {
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
        info!("Failed to connect to MQTT broker, retrying...");
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
                        Err(e) => info!(
                            "Failed to connect MQTT client: {:?}",
                            format!("{:?}", e).as_str()
                        ),
                    }
                }
                Err(e) => {
                    info!("Failed to read from socket: {:?}", e);
                    return;
                }
            };
        }
        Err(e) => info!("Err: {:?}", e),
    }
    match socket.flush().await {
        Ok(_) => info!("Flushed---"),
        Err(e) => info!("Err: {:?}", e),
    };
    while !mqtt_client.is_connected() {
        info!("---> {}", socket.state());
        // info!("MQTT client not connected, waiting...");
        mqtt_client.connect(MQTT_CLIENT_ID, None).unwrap();
        Timer::after(Duration::from_millis(100)).await;
    }
    loop {
        socket.wait_write_ready().await;
        let payload = match sensor_channel.try_receive() {
            Ok(data) => {
                format!(
                    "{{\"sensor_id\": \"{}\", \"temperature\": \"{}\", \"humidity\": \"{}\"}}",
                    MQTT_CLIENT_ID, data.temperature, data.humidity
                )
            }
            Err(_) => {
                format!(
                    "{{\"sensor_id\": \"{}\", \"temperature\": -1, \"humidity\": -1}}",
                    MQTT_CLIENT_ID
                )
            }
        };
        match mqtt_client.publish(MQTT_TOPIC, payload.as_bytes()) {
            Ok(packet) => match socket.write(packet).await {
                Ok(size) => {
                    info!("Publish success with {} bytes", size);
                    socket.flush().await.unwrap();
                }
                Err(e) => info!("Err: {:?}", e),
            },
            Err(e) => {
                let err = format!("{:?}", e);
                info!("Publishing error: {}", err.as_str())
            }
        }
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn dht_task(pin: AnyPin<'static>, sensor_channel: &'static SensorChannel) {
    let mut dht = DHT11::new(pin);
    loop {
        critical_section::with(|_| match dht.read() {
            Ok(data) => {
                sensor_channel.try_send(data).ok();
            }
            Err(error) => {
                let err = format!("{:?}", error);
                info!("DHT11 read error: {}", err.as_str());
            }
        });
        Timer::after(Duration::from_secs(1)).await;
    }
}
