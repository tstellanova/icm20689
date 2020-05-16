# icm20689 

A rust embedded-hal driver for the 
TDK InvenSense 
[ICM-20689](https://invensense.tdk.com/download-pdf/icm-20689-datasheet/)
6DOF accelerometer and gyroscope.

The ICM-20689 is a 6-axis motion tracking device 
that combines a 3-axis gyroscope, 3-axis accelerometer, 
and a motion processor.
This driver supports some similar 6dof devices in the same family, such as:
- ICM-20602
- ICM-20608G

## Status

- [x] Basic SPI support
- [ ] Basic I2C support
- [x] Supported product identifier check
- [x] Read of gyro data
- [x] Read of accel data
- [x] Support for ICM-20608G (tested)
- [ ] Support for ICM-20602 (implemented but untested)
- [ ] Support for DMA with SPI
- [ ] Tests with mock embedded hal
- [ ] Usage example with `cortex-m` hal
- [ ] Doc comments
- [ ] CI
- [ ] Support for user recalibration






