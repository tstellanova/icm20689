pub struct I2cInterface<I2C> {
    /// i2c port
    i2c_port: I2C,
    /// address for i2c communications
    address: u8,
}