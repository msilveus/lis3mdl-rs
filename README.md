# lis3mdl-rs

[//]: # ([![Crates.io][crates-badge]][crates-url])

[//]: # ([![BSD 3-Clause licensed][bsd-badge]][bsd-url])

[//]: # ()
[//]: # ([crates-badge]: https://img.shields.io/crates/v/lis3mdl-rs)

[//]: # ([crates-url]: https://crates.io/crates/lis3mdl-rs)

[//]: # ([bsd-badge]: https://img.shields.io/crates/l/lis3mdl-rs)

[//]: # ([bsd-url]: https://opensource.org/licenses/BSD-3-Clause)

Provides a platform-agnostic, no_std-compatible driver for the ST LIS3MDL sensor, supporting both I2C and SPI communication interfaces.
Note: This driver is modeled after the ST LIS2MDL Rust driver, which can be found [here](https://github.com/STMicroelectronics/lis2mdl-rs). I created this driver because I still have this sensor and wanted to use it with Rust.

It should also be noted that the LIS3MDL is obsolete and the recommended replacement is the LIS2MDL.

## Sensor Overview

The LIS3MDL is an ultralow-power high-performance 3-axis magnetic sensor with user-selectable full scales of ±4/±8/±12/±16 gauss.

The self-test capability allows the user to check the functioning of the sensor in the final application.

The device may be configured to generate interrupt signals for magnetic field detection.

The LIS3MDL includes an I²C serial bus interface that supports standard and fast mode (100 kHz and 400 kHz) and SPI serial standard interfaces.

The device is available in a small thin plastic, land grid array (LGA) package and is guaranteed to operate over an extended temperature range of -40°C to +85°C.

For more info, please visit the device page at [https://www.st.com/en/mems-and-sensors/lis3mdl.html](https://www.st.com/en/mems-and-sensors/lis3mdl.html)

## Installation

Add the driver to your `Cargo.toml` dependencies:

```toml
[dependencies]
lis3mdl-rs = "0.1.0"
```

Or, add it directly from the terminal:

```sh
cargo add lis3mdl-rs
```

## Usage

Include the crate and its prelude
```rust
use lis3mdl_rs as lis3mdl;
use lis3mdl::*;
use lis3mdl::prelude::*;
```

### Create an instance

Create an instance of the driver with the `new_<bus>` associated function, by passing an I2C (`embedded_hal::i2c::I2c`) instance and I2C address, or an SPI (`embedded_hal::spi::SpiDevice`) instance.

An example with I2C:

```rust
let mut sensor = Lis3mdl::new_i2c(i2c, I2CAddress::I2cAdd);
```

### Check "Who Am I" Register

This step ensures correct communication with the sensor. It returns a unique ID to verify the sensor's identity.

```rust
let whoami = sensor.device_id_get().unwrap();
if whoami != ID {
    panic!("Invalid sensor ID");
}
```

### Configure

See details in specific examples; the following are common api calls:

```rust
// Restore default configuration
sensor.reset_set(1).unwrap();

// Wait for reset to complete
while sensor.reset_get().unwrap() != 0 {}

// Enable Block Data Update
sensor.block_data_update_set(1).unwrap();

// Set Output Data Rate
sensor.data_rate_set(Odr::_10hz).unwrap();

// Set / Reset sensor mode
sensor
    .set_rst_mode_set(SetRst::SensOffCancEveryOdr)
    .unwrap();

// Enable temperature compensation
sensor.offset_temp_comp_set(1).unwrap();

// Set device in continuous mode
sensor.operating_mode_set(Md::ContinuousMode).unwrap();
```

## License

Distributed under the BSD-3 Clause license.

More Information: [http://www.st.com](http://st.com/MEMS).

**Copyright (C) 2025 STMicroelectronics**