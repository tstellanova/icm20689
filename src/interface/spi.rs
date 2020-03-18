use embedded_hal as hal;
use hal::blocking::delay::DelayMs;
use hal::digital::v2::OutputPin;

use super::SensorInterface;
use crate::Error;

/// This combines the SPI peripheral and
/// associated control pins such as:
/// - CSN : Chip Select (aka SS or Slave Select)
/// - DRDY: Data Ready: Sensor uses this to indicate it had data available for read
pub struct SpiInterface<SPI, CSN> {
    /// the SPI port to use when communicating
    spi: SPI,
    /// the Chip Select pin (GPIO output) to use when communicating
    csn: CSN,
}

impl<SPI, CSN, CommE, PinE> SpiInterface<SPI, CSN>
where
    SPI: hal::blocking::spi::Write<u8, Error = CommE>
        + hal::blocking::spi::Transfer<u8, Error = CommE>,
    CSN: OutputPin<Error = PinE>,
{
    pub fn new(spi: SPI, csn: CSN) -> Self {
        Self { spi: spi, csn: csn }
    }

    fn transfer_block(&mut self, block: &mut [u8]) -> Result<(), Error<CommE, PinE>> {
        self.csn.set_low().map_err(Error::Pin)?;
        let rc = self.spi.transfer(block);
        self.csn.set_high().map_err(Error::Pin)?;
        let _ = rc.map_err(Error::Comm)?;

        Ok(())
    }
}

impl<SPI, CSN, CommE, PinE> SensorInterface for SpiInterface<SPI, CSN>
where
    SPI: hal::blocking::spi::Write<u8, Error = CommE>
        + hal::blocking::spi::Transfer<u8, Error = CommE>,
    CSN: OutputPin<Error = PinE>,
    //DRDY: InputPin<Error = PinE>,
{
    type InterfaceError = Error<CommE, PinE>;

    fn setup(&mut self, _delay_source: &mut impl DelayMs<u8>) -> Result<(), Self::InterfaceError> {
        // Deselect sensor
        self.csn.set_high().map_err(Error::Pin)?;

        Ok(())
    }

    fn register_read(&mut self, reg: u8) -> Result<u8, Self::InterfaceError> {
        /// Combined with register address for reading single byte register
        const DIR_READ: u8 = 0x80;

        let mut cmd: [u8; 2] = [reg | DIR_READ, 0];
        self.transfer_block(&mut cmd)?;
        Ok(cmd[1])
    }

    fn register_write(&mut self, reg: u8, val: u8) -> Result<(), Self::InterfaceError> {
        let mut cmd: [u8; 2] = [reg, val];
        self.transfer_block(&mut cmd)?;

        Ok(())
    }
}
