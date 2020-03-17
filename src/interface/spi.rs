

use crate::Error;

use embedded_hal as hal;
use hal::digital::v2::{InputPin, OutputPin};

use super::SensorInterface;

/// This combines the SPI peripheral and
/// associated control pins such as:
/// - CSN : Chip Select (aka SS or Slave Select)
/// - HINTN: Hardware Interrupt. Sensor uses this to indicate it had data available for read
/// - WAK: Wake pin.  Master asserts this to choose SPI mode, then deasserts to wake up the sensor.
pub struct SpiInterface<SPI, CSN, DRDY> {
    /// the SPI port to use when communicating
    spi: SPI,
    /// the Chip Select pin (GPIO output) to use when communicating
    csn: CSN,
    /// an input pin (GPIO input) that indicates when data is available (Data Ready)
    drdy: DRDY,
}

impl<SPI, CSN, DRDY, CommE, PinE> SpiInterface<SPI, CSN, DRDY>
    where
        SPI: embedded_hal::blocking::spi::Write<u8, Error = CommE>
        + embedded_hal::blocking::spi::Transfer<u8, Error = CommE>,
        CSN: OutputPin<Error = PinE>,
        DRDY: InputPin<Error = PinE>,
{
    pub fn new(spi: SPI, csn: CSN, drdy: DRDY) -> Self {
        Self {
            spi,
            csn,
            drdy,
        }
    }
}