pub mod i2c;
pub mod spi;

pub use self::i2c::I2cInterface;
pub use self::spi::SpiInterface;

/// A method of communicating with the sensor
pub trait SensorInterface {
    /// Interface associated error type
    type InterfaceError;

    /// Read a single byte from a register address
    fn register_read(&mut self, reg: u8) -> Result<u8, Self::InterfaceError>;
    /// Write a single byte to a register address
    fn register_write(&mut self, reg: u8, val: u8) -> Result<(), Self::InterfaceError>;

    /// read a vector of three i16 from the given register address
    fn read_vec3_i16(&mut self, reg: u8) -> Result<[i16; 3], Self::InterfaceError>;
}
