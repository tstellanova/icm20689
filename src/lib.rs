/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

use embedded_hal as hal;
use hal::digital::v2::{InputPin, OutputPin};

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
    {
        let iface = interface::I2cInterface::new(i2c, address);
        ICM20689::new_with_interface(iface)
    }

    /// Create a new driver using SPI interface
    pub fn new_spi<SPI, CSN, CommE, PinE>(
        spi: SPI,
        csn: CSN,
    ) -> ICM20689<SpiInterface<SPI, CSN>>
    where
        SPI: hal::blocking::spi::Transfer<u8, Error = CommE>
            + hal::blocking::spi::Write<u8, Error = CommE>,
        CSN: OutputPin<Error = PinE>,
        // DRDY: InputPin<Error = PinE>,
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
{
    const REG_WHO_AM_I: u8 = 0x75;
    const EXPECTED_WHO_AM_I: u8 = 0x98;

    pub(crate) fn new_with_interface(sensor_interface: SI) -> Self {
        Self { sensor_interface }
    }

    /// Read the sensor identifier and
    /// return true if it matches the expected value
    pub fn probe(&mut self) -> bool {
        let rc = self.sensor_interface.register_read(Self::REG_WHO_AM_I);
        let val = rc.unwrap_or(0);
        val == Self::EXPECTED_WHO_AM_I
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
