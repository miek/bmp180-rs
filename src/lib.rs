//! Driver for Bosch BMP180 digital barometric pressure sensors

#![no_std]
#[cfg(test)]
#[macro_use]
extern crate std;

extern crate byteorder;
extern crate embedded_hal;

use byteorder::{ByteOrder, BigEndian};
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::i2c::{Write, WriteRead};

static ADDRESS: u8 = 0x77;

pub struct BMP180<I2C, D> {
    i2c: I2C,
    delay: D,
    calibration: Calibration,
    variables: Variables,
}

impl<I2C, D, E> BMP180<I2C, D>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
    D: DelayUs<u16>,
{
	/// Creates a new driver
    pub fn new(i2c: I2C, delay: D) -> Result<Self, Error<E>> {
        let calibration: Calibration = Default::default();
        let mut bmp = BMP180 { i2c, delay, calibration: calibration, variables: Default::default() };
        let mut buf = [0; 22];
        bmp.read_reg(0xAA, &mut buf)?;
        bmp.calibration = Calibration{
            ac1: BigEndian::read_i16(&buf[0..2]),
            ac2: BigEndian::read_i16(&buf[2..4]),
            ac3: BigEndian::read_i16(&buf[4..6]),
            ac4: BigEndian::read_u16(&buf[6..8]),
            ac5: BigEndian::read_u16(&buf[8..10]),
            ac6: BigEndian::read_u16(&buf[10..12]),
            b1:  BigEndian::read_i16(&buf[12..14]),
            b2:  BigEndian::read_i16(&buf[14..16]),
            mb:  BigEndian::read_i16(&buf[16..18]),
            mc:  BigEndian::read_i16(&buf[18..20]),
            md:  BigEndian::read_i16(&buf[20..22]),
        };
        Ok(bmp)
    }

    fn calculate_temperature(&mut self, ut: u16) -> i32 {
        let cal = &self.calibration;
        let vars = &mut self.variables;
        vars.x1 = ((ut as i32 - cal.ac6 as i32) * cal.ac5 as i32) >> 15;
        vars.x2 = ((cal.mc as i32) << 11) / (vars.x1 as i32 + cal.md as i32) -1;
        vars.b5 = vars.x1 + vars.x2;
        (vars.b5 + 8) >> 4
    }

    fn calculate_pressure(&mut self, up: u16, oss: Oversampling) -> i32 {
        let cal = &self.calibration;
        let vars = &mut self.variables;
        vars.b6 = vars.b5 - 4000;
        vars.x1 = (cal.b2 as i32 * ((vars.b6 * vars.b6) >> 12)) >> 11;
        vars.x2 = (cal.ac2 as i32 * vars.b6) >> 11;
        vars.x3 = vars.x1 + vars.x2;
        vars.b3 = (((cal.ac1 as i32 * 4 + vars.x3) << oss as i32) + 2) / 4;
        vars.x1 = (cal.ac3 as i32 * vars.b6) >> 13;
        vars.x2 = (cal.b1 as i32 * ((vars.b6 * vars.b6) >> 12)) >> 16;
        vars.x3 = (vars.x1 + vars.x2 + 2) >> 2;
        vars.b4 = (cal.ac4 as u32 * (vars.x3 + 32768) as u32) >> 15;
        vars.b7 = (up as u32 - vars.b3 as u32) * (50000 >> oss as u32);
        let mut p = if vars.b7 < 0x8000_0000 {
            (vars.b7 * 2) / vars.b4
        } else {
            (vars.b7 / vars.b4) * 2 
        } as i32;
        vars.x1 = (p >> 8) * (p >> 8);
        vars.x1 = (vars.x1 * 3038) >> 16;
        vars.x2 = (-7357 * p) >> 16;
        p = p + ((vars.x1 + vars.x2 + 3791) >> 4);
        p as i32
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(ADDRESS, &[reg, value])
            .map_err(Error::I2c)
    }

    fn read_reg(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c
            .write_read(ADDRESS, &[reg], buf)
            .map_err(Error::I2c)
    }

    fn measure(&mut self, command: Command) -> Result<u16, Error<E>> {
        self.write_reg(0xF4, command.value())?;
        self.delay.delay_us(command.max_duration());

        let mut buf = [0; 2];
        self.read_reg(0xF6, &mut buf)?;
        Ok(BigEndian::read_u16(&buf))
    }

    pub fn temperature(&mut self) -> Result<i32, Error<E>> {
        let ut = self.measure(Command::Temperature)?;
        Ok(self.calculate_temperature(ut))
    }

    pub fn temperature_and_pressure(&mut self, oss: Oversampling) -> Result<(i32, i32), Error<E>> {
        // Temp reading must be done first to get `b5` for pressure calc
        let temp = self.temperature()?;
        let ut = self.measure(Command::Pressure(oss))?;
        Ok((temp, self.calculate_pressure(ut, oss)))
    }

    pub fn destroy(self) -> I2C {
        self.i2c
    }
}

/// Errors
#[derive(Debug)]
pub enum Error<E> {
    /// Wrong CRC
    Crc,
    /// I2C bus error
    I2c(E),
}

#[derive(Debug, Default)]
struct Calibration {
    ac1: i16,
    ac2: i16,
    ac3: i16,
    ac4: u16,
    ac5: u16,
    ac6: u16,
    b1:  i16,
    b2:  i16,
    mb:  i16,
    mc:  i16,
    md:  i16,
}

#[derive(Debug, Default)]
struct Variables {
    x1: i32,
    x2: i32,
    x3: i32,
    b3: i32,
    b4: u32,
    b5: i32,
    b6: i32,
    b7: u32,
}

enum Command {
    Temperature,
    Pressure(Oversampling),
}

#[derive(Clone, Copy)]
pub enum Oversampling {
    O1 = 0,
    O2 = 1,
    O4 = 2,
    O8 = 3,
}

impl Command {
    fn value(&self) -> u8 {
        match *self {
            // Table 8: Control registers values for different internal oversampling_setting (oss)
            Command::Temperature   => 0x2E,
            Command::Pressure(oss) => 0x34 + ((oss as u8) << 6),
        }
    }

    /// Maximum measurement duration in microseconds
    fn max_duration(&self) -> u16 {
        use Oversampling::*;
        match *self {
            Command::Temperature    => 4500,
            Command::Pressure(O1)   => 4500,
            Command::Pressure(O2)   => 7500,
            Command::Pressure(O4)   => 13500,
            Command::Pressure(O8)   => 25500,
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate embedded_hal_mock;
    use super::*;
    use byteorder::{BigEndian, WriteBytesExt};
    use tests::embedded_hal_mock::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
    use tests::embedded_hal_mock::delay::MockNoop as DelayMock;
    #[test]
    fn test_bmp180() {
        let mut cal = vec![];
        cal.write_i16::<BigEndian>(408).unwrap();
        cal.write_i16::<BigEndian>(-72).unwrap();
        cal.write_i16::<BigEndian>(-14383).unwrap();
        cal.write_u16::<BigEndian>(32741).unwrap();
        cal.write_u16::<BigEndian>(32757).unwrap();
        cal.write_u16::<BigEndian>(23153).unwrap();
        cal.write_i16::<BigEndian>(6190).unwrap();
        cal.write_i16::<BigEndian>(4).unwrap();
        cal.write_i16::<BigEndian>(-32768).unwrap();
        cal.write_i16::<BigEndian>(-8711).unwrap();
        cal.write_i16::<BigEndian>(2868).unwrap();

        let mut temp = vec![];
        temp.write_u16::<BigEndian>(27898).unwrap();

        let mut pressure = vec![];
        pressure.write_u16::<BigEndian>(23843).unwrap();

        let expectations = [
            // Read calibration
            I2cTransaction::write_read(ADDRESS, vec![0xAA], cal),

            // Start temp measurement
            I2cTransaction::write(ADDRESS, vec![0xF4, 0x2E]),
            // Read temp measurement
            I2cTransaction::write_read(ADDRESS, vec![0xF6], temp.clone()),

            // Start temp measurement
            I2cTransaction::write(ADDRESS, vec![0xF4, 0x2E]),
            // Read temp measurement
            I2cTransaction::write_read(ADDRESS, vec![0xF6], temp.clone()),
            // Start pressure measurement
            I2cTransaction::write(ADDRESS, vec![0xF4, 0x34]),
            // Read pressure measurement
            I2cTransaction::write_read(ADDRESS, vec![0xF6], pressure),
        ];
        let i2c = I2cMock::new(&expectations);
        let mut bmp = BMP180::new(i2c, DelayMock{}).unwrap();
        let temp = bmp.temperature().unwrap();
        assert_eq!(temp, 150);
        assert_eq!(bmp.variables.x1, 4743);
        assert_eq!(bmp.variables.x2, -2344);
        assert_eq!(bmp.variables.b5, 2399);

        let (temp, pressure) = bmp.temperature_and_pressure(Oversampling::O1).unwrap();
        assert_eq!(bmp.variables.b6, -1601);
        assert_eq!(bmp.variables.x1, 3454);
        assert_eq!(bmp.variables.x2, -7859);
        assert_eq!(bmp.variables.x3, 717);
        assert_eq!(bmp.variables.b3, 422);
        assert_eq!(bmp.variables.b4, 33457);
        assert_eq!(bmp.variables.b7, 1171050000);
        assert_eq!(temp, 150);
        assert_eq!(pressure, 69964);
        let mut i2c = bmp.destroy();
        i2c.done();
    }

    #[test]
    fn check_commands() -> () {
        assert_eq!(Command::Temperature.value(), 0x2E);
        assert_eq!(Command::Pressure(Oversampling::O1).value(), 0x34);
        assert_eq!(Command::Pressure(Oversampling::O2).value(), 0x74);
        assert_eq!(Command::Pressure(Oversampling::O4).value(), 0xB4);
        assert_eq!(Command::Pressure(Oversampling::O8).value(), 0xF4);
    }
}
