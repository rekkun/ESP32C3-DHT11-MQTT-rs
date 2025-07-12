// src/dht_esp.rs

use critical_section::with;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin}; // ✅ Thêm InputPin từ embedded-hal
use esp_hal::gpio::Level::{High, Low};
use esp_hal::gpio::Pin;

// Sử dụng các thành phần từ gốc của crate (tức là từ lib.rs)
use crate::{
    humidity, temperature, DHTSensorError, DTHResponse, HIGH_LEVEL_THRESHOLD, LOW_LEVEL_THRESHOLD,
    WAIT_FOR_READINESS_LEVEL_THRESHOLD,
};

pub struct DHTSensor<'a, P, D>
where
    P: Pin,
    D: DelayNs,
{
    pin: &'a mut P,
    delay: D,
    last_response: Option<DTHResponse>,
}

impl<'a, P, D> DHTSensor<'a, P, D>
where
    // Thêm ràng buộc cho cả InputPin và OutputPin từ embedded_hal
    // Ta cũng thêm ràng buộc Error phải giống nhau và Debug được để dễ xử lý.
    P: Pin + esp_hal::gpio::InputPin + esp_hal::gpio::OutputPin + InputPin + OutputPin,
    D: DelayNs,
{
    pub fn new(pin: &'a mut P, delay: D) -> Self {
        DHTSensor {
            pin,
            delay,
            last_response: None,
        }
    }

    // ... hàm read() không thay đổi ...
    pub fn read(&mut self) -> Result<DTHResponse, DHTSensorError> {
        match self.read_raw_data() {
            Ok(data) => {
                let humidity_data: &[u16; 2] = &data[0..2].try_into().unwrap();
                let humidity = humidity(humidity_data);

                let temperature_data: &[u16; 2] = &data[2..4].try_into().unwrap();
                let temperature = temperature(temperature_data);

                if humidity <= 100.0 {
                    let response = DTHResponse {
                        humidity,
                        temperature,
                    };
                    self.last_response = Some(response.clone());
                    Ok(response)
                } else {
                    self.last_response
                        .clone()
                        .ok_or(DHTSensorError::InvalidData)
                }
            }
            Err(e) => self.last_response.clone().ok_or(e),
        }
    }

    fn read_raw_data(&mut self) -> Result<[u16; 5], DHTSensorError> {
        let mut data: [u16; 5] = [0; 5];
        let mut all_bits_cycles: [u32; 80] = [0; 80];

        with(|_cs| {
            self.pin.set_low().ok();
            self.delay.delay_ms(20);
            self.pin.set_high().ok();
            self.delay.delay_us(30);

            self.wait_while_level(High, WAIT_FOR_READINESS_LEVEL_THRESHOLD)?;
            self.wait_while_level(Low, WAIT_FOR_READINESS_LEVEL_THRESHOLD)?;

            for i in 0..40 {
                self.wait_while_level(High, LOW_LEVEL_THRESHOLD)?;
                all_bits_cycles[i] = self.wait_while_level(Low, HIGH_LEVEL_THRESHOLD)?;
            }
            Ok(())
        })?;

        for i in 0..40 {
            let high_cycles = all_bits_cycles[i];
            data[i / 8] <<= 1;
            if high_cycles > 40 {
                data[i / 8] |= 1;
            }
        }

        let sum = (data[0] as u32 + data[1] as u32 + data[2] as u32 + data[3] as u32) as u16;

        if data[4] == (sum & 0x00FF) as u16 {
            Ok(data)
        } else {
            Err(DHTSensorError::ChecksumError)
        }
    }

    // ✅ SỬA LẠI HÀM NÀY
    fn wait_while_level(
        &mut self,
        level: esp_hal::gpio::Level,
        timeout_us: u32,
    ) -> Result<u32, DHTSensorError> {
        let mut elapsed_us = 0;
        loop {
            // Kiểm tra xem mức pin hiện tại có khớp với mức đang chờ không
            let is_at_level = match level {
                // unwrap() ở đây là ổn vì lỗi đọc pin GPIO hiếm khi xảy ra
                // và không thể phục hồi trong ngữ cảnh này.
                High => self.pin.is_high().unwrap(),
                Low => self.pin.is_low().unwrap(),
            };

            if !is_at_level {
                // Nếu mức pin đã thay đổi, thoát vòng lặp
                break;
            }

            if elapsed_us >= timeout_us {
                return Err(DHTSensorError::Timeout);
            }

            self.delay.delay_us(1);
            elapsed_us += 1;
        }
        Ok(elapsed_us)
    }
}
