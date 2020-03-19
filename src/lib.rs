/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

use embedded_hal as hal;
use hal::digital::v2::OutputPin;

mod interface;
pub use interface::{I2cInterface, SensorInterface, SpiInterface};

/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE, PinE> {
    /// Sensor communication error
    Comm(CommE),
    /// Pin setting error
    Pin(PinE),
}

pub struct Builder {}

impl Builder {
    /// Create a new driver using I2C interface
    pub fn new_i2c<I2C, CommE>(&self, i2c: I2C, address: u8) -> ICM20689<I2cInterface<I2C>>
    where
        I2C: hal::blocking::i2c::Write<Error = CommE>
            + hal::blocking::i2c::Read<Error = CommE>
            + hal::blocking::i2c::WriteRead<Error = CommE>,
        CommE: core::fmt::Debug,
    {
        let iface = interface::I2cInterface::new(i2c, address);
        ICM20689::new_with_interface(iface)
    }

    /// Create a new driver using SPI interface
    pub fn new_spi<SPI, CSN, CommE, PinE>(spi: SPI, csn: CSN) -> ICM20689<SpiInterface<SPI, CSN>>
    where
        SPI: hal::blocking::spi::Transfer<u8, Error = CommE>
            + hal::blocking::spi::Write<u8, Error = CommE>,
        CSN: OutputPin<Error = PinE>,
        CommE: core::fmt::Debug,
        PinE: core::fmt::Debug,
    {
        let iface = interface::SpiInterface::new(spi, csn);
        ICM20689::new_with_interface(iface)
    }
}

pub struct ICM20689<SI> {
    pub(crate) sensor_interface: SI,
}

impl<SI> ICM20689<SI>
where
    SI: SensorInterface,
    SI::InterfaceError: core::fmt::Debug,
{
    const REG_WHO_AM_I: u8 = 0x75;
    const EXPECTED_WHO_AM_I: u8 = 0x98;

    const REG_ACCEL_XOUT_H: u8 = 0x3B;
    const REG_ACCEL_START: u8 = Self::REG_ACCEL_XOUT_H;

    const REG_GYRO_XOUT_H: u8 = 0x43;
    const REG_GYRO_START: u8 = Self::REG_GYRO_XOUT_H;

    pub(crate) fn new_with_interface(sensor_interface: SI) -> Self {
        Self { sensor_interface }
    }

    /// Read the sensor identifier and
    /// return true if it matches the expected value
    pub fn probe(&mut self) -> Result<bool, SI::InterfaceError> {
        //dummy read may wake up the sensor
        self.sensor_interface.register_read(Self::REG_WHO_AM_I)?;
        let val = self.sensor_interface.register_read(Self::REG_WHO_AM_I)?;

        Ok(val == Self::EXPECTED_WHO_AM_I)
    }

    pub fn get_accel(&mut self) -> Result<[i16; 3], SI::InterfaceError> {
        let sample = self.sensor_interface.read_vec3_i16(Self::REG_ACCEL_START)?;
        Ok(sample)
    }

    pub fn get_gyro(&mut self) -> Result<[i16; 3], SI::InterfaceError> {
        let sample = self.sensor_interface.read_vec3_i16(Self::REG_GYRO_START)?;
        Ok(sample)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
