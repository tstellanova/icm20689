/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

use embedded_hal as hal;
use hal::delay::DelayNs;
use hal::digital::OutputPin;

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

mod interface;
pub use interface::{I2cInterface, SensorInterface, SpiInterface};

/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE, PinE> {
    /// Sensor communication error
    Comm(CommE),
    /// Pin setting error
    Pin(PinE),

    /// Unrecognized chip ID
    UnknownChipId,
    /// Sensor not responding
    Unresponsive,
}

pub struct Builder {}

impl Builder {
    /// Create a new driver using I2C interface
    pub fn new_i2c<I2C, CommE>(&self, i2c: I2C, address: u8) -> ICM20689<I2cInterface<I2C>>
    where
        I2C: hal::i2c::I2c<Error = CommE>,
        CommE: core::fmt::Debug,
    {
        let iface = interface::I2cInterface::new(i2c, address);
        ICM20689::new_with_interface(iface)
    }

    /// Create a new driver using SPI interface
    pub fn new_spi<SPI, CSN, CommE, PinE>(spi: SPI, csn: CSN) -> ICM20689<SpiInterface<SPI, CSN>>
    where
        SPI: hal::spi::SpiDevice<u8, Error = CommE>,
        CSN: OutputPin<Error = PinE>,
        CommE: core::fmt::Debug,
        PinE: core::fmt::Debug,
    {
        let iface = interface::SpiInterface::new(spi, csn);
        ICM20689::new_with_interface(iface)
    }
}

pub struct ICM20689<SI> {
    pub(crate) si: SI,

    pub(crate) gyro_scale: f32,
    pub(crate) accel_scale: f32,
}

impl<SI, CommE, PinE> ICM20689<SI>
where
    SI: SensorInterface<InterfaceError = Error<CommE, PinE>>,
{
    pub(crate) fn new_with_interface(sensor_interface: SI) -> Self {
        Self {
            si: sensor_interface,
            gyro_scale: 0.0,
            accel_scale: 0.0
        }
    }

    /// Read the sensor identifier and return true if they match a supported value
    pub fn check_identity(
        &mut self,
        delay_source: &mut impl DelayNs,
    ) -> Result<bool, SI::InterfaceError> {
        for _ in 0..5 {
            let chip_id = self.si.register_read(REG_WHO_AM_I)?;
            match chip_id {
                ICM20602_WAI | ICM20608_WAI | ICM20689_WAI => {
                    #[cfg(feature = "rttdebug")]
                    rprintln!("found device: 0x{:0x}  ", chip_id);
                    return Ok(true);
                }
                _ => {
                    #[cfg(feature = "rttdebug")]
                    rprintln!("bogus whoami: 0x{:0x}  ", chip_id);
                }
            }

            delay_source.delay_ms(10);
        }

        Ok(false)
    }

    /// Perform a soft reset on the sensor
    pub fn soft_reset(
        &mut self,
        delay_source: &mut impl DelayNs,
    ) -> Result<(), SI::InterfaceError> {
        /// disable I2C interface if we're using SPI
        const I2C_IF_DIS: u8 = 1 << 4;

        /// reset the device
        const PWR_DEVICE_RESET: u8 = 1 << 7; // 0x80 : 0b10000000;

        /// Auto-select between internal relaxation oscillator and
        /// gyroscope MEMS oscillator to use the best available source
        const CLKSEL_AUTO: u8 = 0x01;
        const SENSOR_ENABLE_ALL: u8 = 0x00;

        // self.dev.write(Register::PWR_MGMT_1, 0x80)?;
        // // get stable time source;
        // // Auto select clock source to be PLL gyroscope reference if ready
        // // else use the internal oscillator, bits 2:0 = 001
        // self.dev.write(Register::PWR_MGMT_1, 0x01)?;
        // // Enable all sensors
        // self.dev.write(Register::PWR_MGMT_2, 0x00)?;
        // delay.delay_ms(200);

        self.si.register_write(REG_PWR_MGMT_1, PWR_DEVICE_RESET)?;
        //reset can take up to 100 ms?
        delay_source.delay_ms(110);

        let mut reset_success = false;
        for _ in 0..10 {
            //The reset bit automatically clears to 0 once the reset is done.
            if let Ok(reg_val) = self.si.register_read(REG_PWR_MGMT_1) {
                if reg_val & PWR_DEVICE_RESET == 0 {
                    reset_success = true;
                    break;
                }
            }
            delay_source.delay_ms(10);
        }
        if !reset_success {
            #[cfg(feature = "rttdebug")]
            rprintln!("couldn't read REG_PWR_MGMT_1");
            return Err(Error::Unresponsive);
        }

        if self.si.using_spi() {
            // disable i2c just after reset
            self.si.register_write(REG_USER_CTRL, I2C_IF_DIS)?;
        }

        //setup the automatic clock selection
        self.si.register_write(REG_PWR_MGMT_1, CLKSEL_AUTO)?;
        //enable accel and gyro
        self.si.register_write(REG_PWR_MGMT_2, SENSOR_ENABLE_ALL)?;

        delay_source.delay_ms(200);

        Ok(())
    }

    /// give the sensor interface a chance to set up
    pub fn setup(&mut self, delay_source: &mut impl DelayNs) -> Result<(), SI::InterfaceError> {
        // const DLPF_CFG_1: u8 = 0x01;
        //const SIG_COND_RST: u8 = 1 << 0;
        const FIFO_RST: u8 = 1 << 2;
        const DMP_RST: u8 = 1 << 3;

        // note that id check before reset will fail
        self.soft_reset(delay_source)?;
        let supported = self.check_identity(delay_source)?;
        if !supported {
            return Err(Error::UnknownChipId);
        }

        //TODO Configure the Digital Low Pass Filter (DLPF)
        // self.si.register_write(Self::REG_CONFIG, DLPF_CFG_1)?;
        // //set the sample frequency
        // self.si.register_write(Self::REG_SMPLRT_DIV, 0x01)?;

        // disable interrupt pin
        self.si.register_write(REG_INT_ENABLE, 0x00)?;

        // disable FIFO
        //self.si.register_write(REG_FIFO_EN, 0x00)?;

        //enable FIFO for gyro and accel only:
        self.si.register_write(REG_FIFO_EN, 0x7C)?;

        //TODO what about SIG_COND_RST  ?
        let ctrl_flags = FIFO_RST | DMP_RST;
        self.si.register_write(REG_USER_CTRL, ctrl_flags)?;

        //configure some default ranges
        self.set_accel_range(AccelRange::default())?;
        self.set_gyro_range(GyroRange::default())?;

        Ok(())
    }

    /// Set the full scale range of the accelerometer
    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), SI::InterfaceError> {
        self.accel_scale = range.scale();
        self.si.register_write(REG_ACCEL_CONFIG, (range as u8) << 3)
    }

    /// Set the full scale range of the gyroscope
    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), SI::InterfaceError> {
        self.gyro_scale = range.scale();
        self.si.register_write(REG_GYRO_CONFIG, (range as u8) << 2)
    }

    pub fn get_raw_accel(&mut self) -> Result<[i16; 3], SI::InterfaceError> {
        self.si.read_vec3_i16(REG_ACCEL_START)
    }

    pub fn get_raw_gyro(&mut self) -> Result<[i16; 3], SI::InterfaceError> {
        self.si.read_vec3_i16(REG_GYRO_START)
    }

    pub fn get_scaled_accel(&mut self) -> Result<[f32; 3], SI::InterfaceError> {
        let raw_accel = self.get_raw_accel()?;
        Ok([
            self.accel_scale * (raw_accel[0] as f32),
            self.accel_scale * (raw_accel[1] as f32),
            self.accel_scale * (raw_accel[2] as f32),
        ])
    }

    pub fn get_scaled_gyro(&mut self) -> Result<[f32; 3], SI::InterfaceError> {
        let raw_gyro = self.get_raw_gyro()?;
        Ok([
            self.gyro_scale * (raw_gyro[0] as f32),
            self.gyro_scale * (raw_gyro[1] as f32),
            self.gyro_scale * (raw_gyro[2] as f32),
        ])
    }

}

/// Common registers
///
const REG_USER_CTRL: u8 = 0x6A;
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_PWR_MGMT_2: u8 = 0x6C;

// const REG_CONFIG: u8 = 0x1A;
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_CONFIG: u8 = 0x1C;

const REG_FIFO_EN: u8 = 0x23;
const REG_INT_ENABLE: u8 = 0x38;
// const REG_SMPLRT_DIV: u8 = 0x19;

const REG_ACCEL_XOUT_H: u8 = 0x3B;
const REG_ACCEL_START: u8 = REG_ACCEL_XOUT_H;

const REG_GYRO_XOUT_H: u8 = 0x43;
const REG_GYRO_START: u8 = REG_GYRO_XOUT_H;

const REG_WHO_AM_I: u8 = 0x75;

/// Device IDs for various supported devices
const ICM20602_WAI: u8 = 0x12;
const ICM20608_WAI: u8 = 0xAF;
const ICM20689_WAI: u8 = 0x98;

#[repr(u8)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// The gyroscope has a programmable full-scale range of ±250, ±500, ±1000, or ±2000 degrees/sec.
pub enum GyroRange {
    /// ±250
    Range_250dps = 0b00,
    /// ±500
    Range_500dps = 0b01,
    /// ±1000
    Range_1000dps = 0b10,
    /// ±2000
    Range_2000dps = 0b11,
}

//Gyro Full Scale Select: 00 = ±250dps
// 01= ±500dps
// 10 = ±1000dps
// 11 = ±2000dps

impl Default for GyroRange {
    fn default() -> Self {
        GyroRange::Range_2000dps
    }
}


impl GyroRange {
    /// convert degrees into radians
    const RADIANS_PER_DEGREE: f32 = core::f32::consts::PI / 180.0;

    /// Gyro range in radians per second per bit
    pub(crate) fn scale(&self) -> f32 {
        Self::RADIANS_PER_DEGREE * self.resolution()
    }

    /// Gyro resolution in degrees per second per bit
    /// Note that the ranges are ± which splits the raw i16 resolution between + and -
    pub(crate) fn resolution(&self) -> f32 {
        match self {
            GyroRange::Range_250dps => 250.0 / 32768.0,
            GyroRange::Range_500dps => 500.0 / 32768.0,
            GyroRange::Range_1000dps => 1000.0 / 32768.0,
            GyroRange::Range_2000dps => 2000.0 / 32768.0,
        }
    }
}

#[repr(u8)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// The accelerometer has a user-programmable accelerometer full-scale range
/// of ±2g, ±4g, ±8g, and ±16g.
/// (g is gravitational acceleration: 9.82 m/s^2)
/// The numeric values of these enums correspond to AFS_SEL
pub enum AccelRange {
    /// ±2g
    Range_2g = 0b00,
    /// ±4g
    Range_4g = 0b01,
    /// ±8g
    Range_8g = 0b10,
    /// ±16g
    Range_16g = 0b11,
}

impl Default for AccelRange {
    fn default() -> Self {
        AccelRange::Range_8g
    }
}

impl AccelRange {
    /// Earth gravitational acceleration (G) standard, in meters per second squared
    const EARTH_GRAVITY_ACCEL: f32 = 9.807;

    /// accelerometer scale in meters per second squared per bit
    pub(crate) fn scale(&self) -> f32 {
        Self::EARTH_GRAVITY_ACCEL * self.resolution()
    }

    /// Accelerometer resolution in G / bit
    /// Note that the ranges are ± which splits the raw i16 resolution between + and -
    pub(crate) fn resolution(&self) -> f32 {
        match self {
            Self::Range_2g => 2.0 / 32768.0,
            Self::Range_4g => 4.0 / 32768.0,
            Self::Range_8g => 8.0 / 32768.0,
            Self::Range_16g => 16.0 / 32768.0,
        }
    }
}
