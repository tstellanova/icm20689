/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

use embedded_hal as hal;
use hal::blocking::delay::DelayMs;
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

    /// Sensor not responding
    Unresponsive,
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

impl<SI, CommE, PinE> ICM20689<SI>
where
    SI: SensorInterface<InterfaceError = Error<CommE, PinE>>,
{
    const REG_PWR_MGMT_1: u8 = 0x6B;
    const PWR_DEVICE_RESET: u8 = 1 << 7; // 0x80 : 0b10000000;
    //const PWR_DEVICE_SLEEP: u8 = 1 << 6; // 0x40
    //const REG_PWR_MGMT_2: u8 = 0x6C;

    const REG_WHO_AM_I: u8 = 0x75;
    const EXPECTED_WHO_AM_I: u8 = 0x98;

    const REG_ACCEL_XOUT_H: u8 = 0x3B;
    const REG_ACCEL_START: u8 = Self::REG_ACCEL_XOUT_H;

    const REG_GYRO_XOUT_H: u8 = 0x43;
    const REG_GYRO_START: u8 = Self::REG_GYRO_XOUT_H;

    pub(crate) fn new_with_interface(sensor_interface: SI) -> Self {
        Self { sensor_interface }
    }

    /// Read the sensor identifiers and
    /// return true if they match the expected value
    pub fn probe(
        &mut self,
        delay_source: &mut impl DelayMs<u8>,
    ) -> Result<bool, SI::InterfaceError> {
        let mut chip_id = 0;
        for _ in 0..5 {
            chip_id = self.sensor_interface.register_read(Self::REG_WHO_AM_I)?;
            if chip_id == Self::EXPECTED_WHO_AM_I {
                break;
            }
            delay_source.delay_ms(10);
        }

        Ok(chip_id == Self::EXPECTED_WHO_AM_I)
    }

    /// Perform a soft reset on the sensor
    pub fn soft_reset(
        &mut self,
        delay_source: &mut impl DelayMs<u8>,
    ) -> Result<(), SI::InterfaceError> {
        self.sensor_interface
            .register_write(Self::REG_PWR_MGMT_1, Self::PWR_DEVICE_RESET)?;
        //reset can take up to 100 ms?
        delay_source.delay_ms(100);
        let mut reset_success = false;
        for _ in 0..100 {
            //The reset bit automatically clears to 0 once the reset is done.
            if let Ok(reg_val) = self.sensor_interface.register_read(Self::REG_PWR_MGMT_1) {
                if reg_val & Self::PWR_DEVICE_RESET == 0 {
                    reset_success = true;
                    break;
                }
            }
            delay_source.delay_ms(10);
        }

        // TODO no need to verify WHO_AM_I in reset?
        // for _ in 0..10 {
        //     let chip_id = self.sensor_interface.register_read(Self::REG_WHO_AM_I)?;
        //     if chip_id == Self::EXPECTED_WHO_AM_I {
        //         let pwr1 = self.sensor_interface.register_read(Self::REG_PWR_MGMT_1)?;
        //         if pwr1 == Self::PWR_DEVICE_SLEEP {
        //             reset_success = true;
        //             break;
        //         }
        //     }
        //     delay_source.delay_ms(1);
        // }

        if reset_success {
            Ok(())
        } else {
            Err(Error::Unresponsive)
        }
    }

    /// give the sensor interface a chance to set up
    pub fn setup(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<(), SI::InterfaceError> {
        let mut probe_ok = self.probe(delay_source)?;
        if !probe_ok {
            self.soft_reset(delay_source)?;
            probe_ok = self.probe(delay_source)?;
        }

        if probe_ok {

        }

        Ok(())
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
