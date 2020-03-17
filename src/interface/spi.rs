

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
        SPI: hal::blocking::spi::Write<u8, Error = CommE>
        + hal::blocking::spi::Transfer<u8, Error = CommE>,
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

impl<SPI, CSN, DRDY, CommE, PinE> SensorInterface for SpiInterface<SPI, CSN, DRDY>
    where
        SPI: hal::blocking::spi::Write<u8, Error = CommE>
        + hal::blocking::spi::Transfer<u8, Error = CommE>,
        CSN: OutputPin<Error = PinE>,
        DRDY: InputPin<Error = PinE>,
{
    type SensorError = Error<CommE, PinE>;

    fn setup(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<(), Self::SensorError> {
        // Deselect sensor
        self.csn.set_high().map_err(Error::Pin)?;

        Ok(())
    }

    fn register_read(&mut self, reg: u8) -> Result<u8, Self::SensorError> {
        self.csn.set_low().map_err(Error::Pin)?;
        let mut read_cmd: [u8; 1] = [ reg | DIR_READ ];
        let val = self
            .spi
            .transfer(&mut read_cmd)
            .map_err(Error::Comm)?;

        //(&mut self, words: &'w mut [W]) -> Result<&'w [W], Self::Error>;

        Ok(val[0])
    }


    // uint8_t ICM20689::RegisterRead(Register reg)
    // {
    // uint8_t cmd[2] {};
    // cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
    // transfer(cmd, cmd, sizeof(cmd));
    // return cmd[1];
    // }
}