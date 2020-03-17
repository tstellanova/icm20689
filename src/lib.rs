/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

use embedded_hal as hal;
use hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;


pub mod interface;
use interface::{I2cInterface, SpiInterface};

/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE> {
    /// Sensor communication error
    Comm(CommE),
}


pub struct Builder { }

impl Builder {
    pub fn new_i2c<I2C, CommE>(&self, i2c: I2C) -> ICM20689<I2cInterface<I2C>>
        where
            I2C: embedded_hal::blocking::i2c::Write<Error = CommE>
            + embedded_hal::blocking::i2c::Read<Error = CommE>
            + embedded_hal::blocking::i2c::WriteRead<Error = CommE>
    {
        //TODO support i2c interface
        unimplemented!()
    }

    /// Finish the builder and use SPI to communicate with the display
    pub fn new_spi<SPI, CSN, DRDY, CommE, PinE>(
        spi: SPI,
        csn: CSN,
        drdy: DRDY,
    ) -> ICM20689<SpiInterface<SPI, CSN, DRDY>>
        where
            SPI: hal::blocking::spi::Transfer<u8, Error = CommE>
            + hal::blocking::spi::Write<u8, Error = CommE>,
            CSN: OutputPin<Error = PinE>,
            DRDY: InputPin<Error = PinE>
    {
        let spi_iface =  interface::SpiInterface::new(spi, csn, drdy);
        ICM20689::new_with_interface(spi_iface)
    }
}

pub struct ICM20689<SI> {
    pub(crate) sensor_interface: SI,
}

impl<SI> ICM20689<SI> {
    pub fn new_with_interface(sensor_interface: SI) -> Self {
        Self {
            sensor_interface,
        }
    }
}



#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
