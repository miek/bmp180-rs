extern crate linux_embedded_hal as hal;
extern crate bmp180;

use hal::{Delay, I2cdev};
use bmp180::{BMP180, Oversampling};

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let mut bmp180 = BMP180::new(dev, Delay).unwrap();

    loop {
        let (temp, pressure) = bmp180.temperature_and_pressure(Oversampling::O1).unwrap();
        println!("Temp: {:.2}C Pressure: {:.2}hPa", temp as f32 / 10.0, pressure as f32 / 100.0);
    }
}
