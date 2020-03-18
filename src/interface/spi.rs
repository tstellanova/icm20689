use crate::Error;

use embedded_hal as hal;
use hal::blocking::delay::DelayMs;
use hal::digital::v2::{InputPin, OutputPin};

use super::SensorInterface;

/// Combined with register address for reading single byte register
const DIR_READ: u8 = 0x80;

/// This combines the SPI peripheral and
/// associated control pins such as:
/// - CSN : Chip Select (aka SS or Slave Select)
/// - DRDY: Data Ready: Sensor uses this to indicate it had data available for read
pub struct SpiInterface<SPI, CSN> {
    /// the SPI port to use when communicating
    spi: SPI,
    /// the Chip Select pin (GPIO output) to use when communicating
    csn: CSN,
    // an input pin (GPIO input) that indicates when data is available (Data Ready)
    // drdy: DRDY,
}

impl<SPI, CSN> SpiInterface<SPI, CSN>
{
    pub fn new(spi: SPI, csn: CSN) -> Self {
        Self {
            spi: spi,
            csn: csn,
            // drdy: drdy,
        }
    }
}

impl<SPI, CSN, CommE, PinE> SensorInterface for SpiInterface<SPI, CSN>
where
    SPI: hal::blocking::spi::Write<u8, Error = CommE>
        + hal::blocking::spi::Transfer<u8, Error = CommE>,
    CSN: OutputPin<Error = PinE>,
    //DRDY: InputPin<Error = PinE>,
{
    type SensorError = Error<CommE, PinE>;

    fn setup(&mut self, _delay_source: &mut impl DelayMs<u8>) -> Result<(), Self::SensorError> {
        // Deselect sensor
        self.csn.set_high().map_err(Error::Pin)?;

        Ok(())
    }

    fn register_read(&mut self, reg: u8) -> Result<u8, Self::SensorError> {
        self.csn.set_low().map_err(Error::Pin)?;
        let mut read_cmd: [u8; 1] = [reg | DIR_READ];
        let val = self.spi.transfer(&mut read_cmd).map_err(Error::Comm)?;

        self.csn.set_high().map_err(Error::Pin)?;

        Ok(val[0])
    }
}
