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
    /// Combined with register address for reading single byte register
    const DIR_READ: u8 = 0x80;

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

    fn read_vec3_i16(&mut self, reg: u8) -> Result<[i16; 3], Self::InterfaceError> {
        let mut block: [u8; 7] = [0; 7];
        block[0] = reg | Self::DIR_READ;
        self.transfer_block(&mut block)?;

        Ok([
            (block[0] as i16) << 8 | (block[1] as i16),
            (block[2] as i16) << 8 | (block[3] as i16),
            (block[4] as i16) << 8 | (block[5] as i16),
        ])
    }

    fn register_read(&mut self, reg: u8) -> Result<u8, Self::InterfaceError> {
        let mut cmd: [u8; 2] = [reg | Self::DIR_READ, 0];
        self.transfer_block(&mut cmd)?;
        Ok(cmd[1])
    }

    fn register_write(&mut self, reg: u8, val: u8) -> Result<(), Self::InterfaceError> {
        let mut cmd: [u8; 2] = [reg, val];
        self.transfer_block(&mut cmd)?;

        Ok(())
    }
}
