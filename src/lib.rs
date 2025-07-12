#![no_std]

// Khai báo module dht_esp, Rust sẽ tìm file `src/dht_esp.rs`
mod dht_esp;

// Tái xuất khẩu (pub use) các thành phần quan trọng từ module dht_esp
// để main.rs có thể truy cập chúng một cách dễ dàng qua `use ten_du_an::...`
pub use dht_esp::DHTSensor;

// Định nghĩa các kiểu dữ liệu chung ở đây
#[derive(Debug, Clone)]
pub enum DHTSensorError {
    Timeout,
    ChecksumError,
    InvalidData,
}

#[derive(Clone, Debug)]
pub struct DTHResponse {
    pub humidity: f32,
    pub temperature: f32,
}

// Các hằng số này có thể ở đây hoặc trong module dht_esp
const WAIT_FOR_READINESS_LEVEL_THRESHOLD: u32 = 85;
const LOW_LEVEL_THRESHOLD: u32 = 55;
const HIGH_LEVEL_THRESHOLD: u32 = 75;

// Các hàm tính toán này có thể được chuyển vào dht_esp.rs nếu muốn,
// nhưng để ở đây cũng không sao vì chúng không phụ thuộc phần cứng.
fn humidity(data: &[u16; 2]) -> f32 {
    // ... logic tính độ ẩm ...
    ((data[0] << 8) | data[1]) as f32 / 10.0
}

fn temperature(data: &[u16; 2]) -> f32 {
    // ... logic tính nhiệt độ ...
    let mut temp = (((data[0] & 0x7F) << 8) | data[1]) as f32 / 10.0;
    if data[0] & 0x80 != 0 {
        temp = -temp;
    }
    temp
}