use embedded_hal as hal;
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
        let mut inst = Self { spi: spi, csn: csn };
        //ensure that the device is initially deselected
        let _ = inst.csn.set_high();
        inst
    }

    fn transfer_block(&mut self, block: &mut [u8]) -> Result<(), Error<CommE, PinE>> {
        self.csn.set_low().map_err(Error::Pin)?;
        let rc = self.spi.transfer(block);
        let _ = self.csn.set_high();
        rc.map_err(Error::Comm)?;

        Ok(())
    }

}

impl<SPI, CSN, CommE, PinE> SensorInterface for SpiInterface<SPI, CSN>
where
    SPI: hal::blocking::spi::Write<u8, Error = CommE>
        + hal::blocking::spi::Transfer<u8, Error = CommE>,
    CSN: OutputPin<Error = PinE>,
{
    type InterfaceError = Error<CommE, PinE>;

    fn read_vec3_i16(&mut self, reg: u8) -> Result<[i16; 3], Self::InterfaceError> {
        let mut resp: [u8; 6] = [0; 6];
        let mut block: [u8; 7] = [0; 7];
        block[0] = reg | Self::DIR_READ;
        self.transfer_block(&mut block)?;

        resp.copy_from_slice(&block[1..7]);

        Ok([
            (resp[0] as i16) << 8 | (resp[1] as i16),
            (resp[2] as i16) << 8 | (resp[3] as i16),
            (resp[4] as i16) << 8 | (resp[5] as i16),
        ])
    }

    fn register_read(&mut self, reg: u8) -> Result<u8, Self::InterfaceError> {
        let mut cmd: [u8; 2] = [reg | Self::DIR_READ, 0];
        self.csn.set_low().map_err(Error::Pin)?;
        let rc = self.spi.transfer(&mut cmd);
        let _ = self.csn.set_high();
        rc.map_err(Error::Comm)?;

        Ok(cmd[1])
    }

    fn register_write(&mut self, reg: u8, val: u8) -> Result<(), Self::InterfaceError> {
        self.csn.set_low().map_err(Error::Pin)?;
        let rc = self.spi.write(&[reg, val]);
        let _ = self.csn.set_high();
        rc.map_err(Error::Comm)?;
        Ok(())
    }

    fn using_spi(&self) -> bool {
        true
    }
}
