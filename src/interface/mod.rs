pub mod i2c;
pub mod spi;

use embedded_hal::blocking::delay::DelayMs;

pub use self::i2c::I2cInterface;
pub use self::spi::SpiInterface;

/// A method of communicating with the sensor
pub trait SensorInterface {
    /// Interface error type
    type InterfaceError;

    /// give the sensor interface a chance to set up
    fn setup(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<(), Self::InterfaceError>;

    /// Read a single register
    fn register_read(&mut self, reg: u8) -> Result<u8, Self::InterfaceError>;
    fn register_write(&mut self, reg: u8, val: u8) -> Result<(), Self::InterfaceError>;

    //TODO read and write block?
}
