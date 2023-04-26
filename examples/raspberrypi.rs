use icm20689::{AccelRange, Builder, GyroRange};
use linux_embedded_hal::spidev::{self, SpidevOptions};
use linux_embedded_hal::sysfs_gpio::Direction;
use linux_embedded_hal::{Delay, Pin, Spidev};

fn main() {
    let mut spi = Spidev::open("/dev/spidev1.0").expect("SPI device");
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(10_000_000)
        .mode(spidev::SpiModeFlags::SPI_MODE_0)
        .build();
    spi.configure(&options).expect("SPI configuration");

    let cs = Pin::new(16); // CS2 pin
    cs.export().expect("cs export");
    while !cs.is_exported() {}
    cs.set_direction(Direction::Out).expect("CS Direction");
    cs.set_value(1).expect("CS Value set to 1");

    //initialize the sensor thourgh spi
    let mut imu = Builder::new_spi(spi, cs);
    // let mut imu_i2c = Builder::new_i2c(i2c, adress);

    //you need to implement an delay_source
    let mut delay_source = Delay {};

    println!(
        "Check device, device support = {}",
        imu.check_identity(&mut delay_source).unwrap()
    );

    imu.setup(&mut delay_source).expect("error setup");

    imu.set_accel_range(AccelRange::Range_2g)
        .expect("error set_accel");
    imu.set_gyro_range(GyroRange::Range_250dps)
        .expect("error set_gyro");

    let accel = imu.get_scaled_accel().unwrap();
    let gyro = imu.get_scaled_gyro().unwrap();
    println!("Accelerometer: {:?}", accel);
    println!("Gyroscope: {:?}", gyro);
}
