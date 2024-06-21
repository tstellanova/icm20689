use embedded_hal as hal;
use hal::digital::OutputPin;

use super::SensorInterface;
use crate::Error;
#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

/// This combines the SPI peripheral and
/// associated control pins such as:
/// - CSN : Chip Select (aka SS or Slave Select)
pub struct SpiInterface<SPI, CSN> {
    /// the SPI port to use when communicating
    spi: SPI,
    /// the Chip Select pin (GPIO output) to use when communicating
    csn: CSN,
}

impl<SPI, CSN, CommE, PinE> SpiInterface<SPI, CSN>
where
    SPI: hal::spi::SpiDevice<u8, Error = CommE>,
    CSN: OutputPin<Error = PinE>,
{
    /// Combined with register address for reading single byte register
    const DIR_READ: u8 = 0x80; // same as 1<<7

    pub fn new(spi: SPI, csn: CSN) -> Self {
        let mut inst = Self { spi, csn };
        //ensure that the device is initially deselected
        let _ = inst.csn.set_high();
        inst
    }

    /// Release owned resources
    pub fn release(self) -> (SPI, CSN) {
        (self.spi, self.csn)
    }

    fn read_block(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), Error<CommE, PinE>> {
        buffer[0] = reg | Self::DIR_READ;
        self.csn.set_low().map_err(Error::Pin)?;
        let rc = self.spi.read(buffer);
        self.csn.set_high().map_err(Error::Pin)?;
        rc.map_err(Error::Comm)?;

        Ok(())
    }

    fn write_block(&mut self, block: &[u8]) -> Result<(), Error<CommE, PinE>> {
        #[cfg(feature = "rttdebug")]
        rprintln!("write {:x?} ", block);

        self.csn.set_low().map_err(Error::Pin)?;
        let rc = self.spi.write(block);
        self.csn.set_high().map_err(Error::Pin)?;
        rc.map_err(Error::Comm)?;

        Ok(())
    }
}

impl<SPI, CSN, CommE, PinE> SensorInterface for SpiInterface<SPI, CSN>
where
    SPI: hal::spi::SpiDevice<u8, Error = CommE>,
    CSN: OutputPin<Error = PinE>,
{
    type InterfaceError = Error<CommE, PinE>;

    fn read_vec3_i16(&mut self, reg: u8) -> Result<[i16; 3], Self::InterfaceError> {
        let mut block: [u8; 7] = [0; 7];
        self.read_block(reg, &mut block)?;

        Ok([
            (block[1] as i16) << 8 | (block[2] as i16),
            (block[3] as i16) << 8 | (block[4] as i16),
            (block[5] as i16) << 8 | (block[6] as i16),
        ])
    }

    fn register_read(&mut self, reg: u8) -> Result<u8, Self::InterfaceError> {
        let mut block: [u8; 2] = [0; 2];
        self.read_block(reg, &mut block)?;

        #[cfg(feature = "rttdebug")]
        rprintln!("read reg 0x{:x} {:x?} ", reg, block[1]);

        Ok(block[1])
    }

    fn register_write(&mut self, reg: u8, val: u8) -> Result<(), Self::InterfaceError> {
        let block: [u8; 2] = [reg, val];
        self.write_block(&block)?;
        Ok(())
    }

    fn using_spi(&self) -> bool {
        true
    }
}
