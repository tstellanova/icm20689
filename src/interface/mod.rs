pub mod i2c;
pub mod spi;

use embedded_hal::blocking::delay::DelayMs;

pub use self::spi::SpiInterface;
pub use self::i2c::I2cInterface;

/// A method of communicating with the sensor
pub trait SensorInterface {
    /// Interface error type
    type SensorError;

    /// give the sensor interface a chance to set up
    fn setup(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<(), Self::SensorError>;

    // /// Write the whole packet provided
    // fn write_packet(&mut self, packet: &[u8]) -> Result<(), Self::SensorError>;
    //
    // /// Read the next packet from the sensor
    // /// Returns the size of the packet read (up to the size of the slice provided)
    // fn read_packet(&mut self, recv_buf: &mut [u8]) -> Result<usize, Self::SensorError>;

    // /// Send a packet and receive the response immediately
    // fn send_and_receive_packet(
    //     &mut self,
    //     send_buf: &[u8],
    //     recv_buf: &mut [u8],
    // ) -> Result<usize, Self::SensorError>;

    //fn register_read(reg: u8) -> Result<(), Self::SensorError>;
    //fn write_register_block(reg: u8, block: &[u8]) -> Result<(), Self::SensorError>;

}

