#![no_std]
#![doc = include_str!("../README.md")]
pub mod prelude;
pub mod register;

use core::fmt::Debug;
use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;
use prelude::*;
use st_mems_bus::*;

/// Driver for LIS3MDL sensor.
///
/// The struct takes a bus object to write to the registers.
/// The bus is generalized over the BusOperation trait, allowing the use
/// of I2C or SPI protocols; this also allows the user to implement sharing
/// techniques to share the underlying bus.
pub struct Lis3mdl<B> {
    /// The bus driver.
    pub bus: B,
}

/// Driver errors.
#[derive(Debug)]
pub enum Error<B> {
    Bus(B),          // Error at the bus level
    UnexpectedValue, // Unexpected value read from a register
}
impl<B> Lis3mdl<B>
where
    B: BusOperation,
{
    /// Constructor method using a generic Bus that implements BusOperation
    pub fn new_bus(bus: B) -> Self {
        Self { bus }
    }
}
impl<P> Lis3mdl<i2c::I2cBus<P>>
where
    P: I2c,
{
    /// Constructor method for using the I2C bus.
    pub fn new_i2c(i2c: P, address: I2CAddress) -> Self {
        // Initialize the I2C bus with the COMPONENT address
        let bus = i2c::I2cBus::new(i2c, address as SevenBitAddress);
        Self { bus }
    }
}
impl<P> Lis3mdl<spi::SpiBus<P>>
where
    P: SpiDevice,
{
    /// Constructor method for using the SPI bus.
    pub fn new_spi(spi: P) -> Self {
        // Initialize the SPI bus
        let bus = spi::SpiBus::new(spi);
        Self { bus }
    }
}

impl<B: BusOperation> Lis3mdl<B> {
    pub fn write_to_register(&mut self, reg: u8, buf: &[u8]) -> Result<(), Error<B::Error>> {
        self.bus.write_to_register(reg, buf).map_err(Error::Bus)
    }

    pub fn read_from_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<B::Error>> {
        self.bus.read_from_register(reg, buf).map_err(Error::Bus)
    }

    /// Set magnetic user offset for hard-iron to compensate environmental effects.
    ///
    /// Data format is the same of output data raw: two's complement
    /// with 1LSb = 1.5mg. These values act on the magnetic output data
    /// value in order to delete the environmental offset.
    pub fn mag_user_offset_set(&mut self, val: &[i16; 3]) -> Result<(), Error<B::Error>> {
        let offset = OffsetXYZ {
            x: val[0],
            y: val[1],
            z: val[2],
        };
        offset.write(self)?;

        Ok(())
    }

    /// Get magnetic user offset for hard-iron to compensate environmental effects.
    ///
    /// Data format is the same of output data raw: two's complement
    /// with 1LSb = 1.5mg. These values act on the magnetic output data
    /// value in order to delete the environmental offset.
    pub fn mag_user_offset_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = OffsetXYZ::read(self)?;

        Ok([val.x, val.y, val.z])
    }

    /// Set Operating mode.
    pub fn operating_mode_set(&mut self, val: Md) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg3::read(self)?;
        reg.set_md(val as u8);
        reg.write(self)
    }
    /// Get Operating mode.
    pub fn operating_mode_get(&mut self) -> Result<Md, Error<B::Error>> {
        let reg = CtrlReg3::read(self)?;

        let val = Md::try_from(reg.md()).unwrap_or_default();
        Ok(val)
    }
    /// Set Output data rate(ODR).
    pub fn data_rate_set(&mut self, val: Odr) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg1::read(self)?;
        reg.set_odr(val as u8);
        reg.write(self)
    }
    /// Get Output data rate(ODR).
    pub fn data_rate_get(&mut self) -> Result<Odr, Error<B::Error>> {
        let reg = CtrlReg1::read(self)?;
        let val = Odr::try_from(reg.odr()).unwrap_or_default();
        Ok(val)
    }
    /// Enables/Disable high-resolution/low-power mode.
    pub fn power_mode_set(&mut self, val: PowerMode) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg3::read(self)?;
        reg.set_lp(val as u8);
        reg.write(self)?;

        Ok(())
    }
    /// Get high-resolution/low-power mode.
    pub fn power_mode_get(&mut self) -> Result<PowerMode, Error<B::Error>> {
        let reg = CtrlReg3::read(self)?;

        let val = PowerMode::try_from(reg.lp()).unwrap_or_default();
        Ok(val)
    }
    /// Enables/Disables the magnetometer temperature compensation.
    pub fn temp_en_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg1::read(self)?;
        reg.set_temp_en(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get configuration (enable/disable) for magnetometer temperature compensation.
    pub fn temp_en_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CtrlReg1::read(self)?.temp_en();

        Ok(val)
    }
    /// Set Reset mode.
    pub fn set_rst_mode_set(&mut self, val: SetRst) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg2::read(self)?;
        reg.set_soft_reset(val as u8);
        reg.write(self)?;

        Ok(())
    }
    /// Get Reset mode.
    pub fn set_rst_mode_get(&mut self) -> Result<SetRst, Error<B::Error>> {
        let reg = CtrlReg2::read(self)?;

        let val = SetRst::try_from(reg.soft_reset()).unwrap_or_default();
        Ok(val)
    }
    /// Enable/Disable Block data update.
    ///
    /// If val is 1: BDU is active
    pub fn block_data_update_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg5::read(self)?;
        reg.set_bdu(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get Block data update configuration (enable/disable).
    pub fn block_data_update_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CtrlReg5::read(self)?.bdu();

        Ok(val)
    }
    /// Get magnetic data ready data available.
    ///
    /// If event available returns 1.
    pub fn mag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = StatusReg::read(self)?.zyxda();

        Ok(val)
    }
    /// Get magnetic data overrun event.
    ///
    /// 0: no overrun has occurred. 1: a new set of data has overwritten the previous set
    pub fn mag_data_ovr_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = StatusReg::read(self)?.zyxor();

        Ok(val)
    }
    /// Get Magnetic raw value.
    ///
    /// Need conversion using `from_lsb_to_mgauss`
    pub fn magnetic_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = OutXYZ::read(self)?;

        Ok([val.x, val.y, val.z])
    }

    /// Get temperature raw value.
    pub fn temperature_raw_get(&mut self) -> Result<i16, Error<B::Error>> {
        TempOut::read(self).map(|temp| temp.0)
    }

    /// Get Device Id.
    ///
    /// Reads the WHO_AM_I register.
    pub fn device_id_get(&mut self) -> Result<u8, Error<B::Error>> {
        WhoAmI::read(self).map(|who_am_i| who_am_i.0)
    }
    /// Set Software reset.
    ///
    /// If val is 1: Restore the default values in user registers.
    /// Flash register keep their values.
    pub fn reset_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg2::read(self)?;
        reg.set_soft_reset(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get Software reset configuration.
    ///
    /// If set to 1: Restore the default values in user registers.
    pub fn reset_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CtrlReg2::read(self)?.soft_reset();

        Ok(val)
    }
    /// Set Reboot memory content.
    ///
    /// If val is 1: Reload the calibration parameters.
    pub fn boot_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg2::read(self)?;
        reg.set_reboot(val);
        reg.write(self)?;

        Ok(())
    }
    /// Reboot memory content. Reload the calibration parameters.
    pub fn boot_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CtrlReg2::read(self)?.reboot();

        Ok(val)
    }
    /// Set Selftest mode.
    pub fn self_test_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg1::read(self)?;
        reg.set_st(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get Selftest configuration.
    pub fn self_test_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CtrlReg1::read(self)?.st();

        Ok(val)
    }
    /// Set Big/Little Endian data format.
    pub fn data_format_set(&mut self, val: Ble) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg4::read(self)?;
        reg.set_ble(val as u8);
        reg.write(self)?;

        Ok(())
    }
    /// Get Big/Little Endian data format.
    pub fn data_format_get(&mut self) -> Result<Ble, Error<B::Error>> {
        let reg = CtrlReg4::read(self)?;

        let val = Ble::try_from(reg.ble()).unwrap_or_default();
        Ok(val)
    }
    /// Get info about device status.
    pub fn status_get(&mut self) -> Result<StatusReg, Error<B::Error>> {
        let val = StatusReg::read(self)?;

        Ok(val)
    }
    /// Set Interrupt generator configuration register.
    pub fn int_gen_conf_set(&mut self, val: &IntCfgReg) -> Result<(), Error<B::Error>> {
        val.write(self)
    }
    /// Get Interrupt generator configuration register.
    pub fn int_gen_conf_get(&mut self) -> Result<IntCfgReg, Error<B::Error>> {
        IntCfgReg::read(self)
    }
    /// Get Interrupt generator source register.
    pub fn int_gen_source_get(&mut self) -> Result<IntSourceReg, Error<B::Error>> {
        IntSourceReg::read(self)
    }
    /// Set User-defined threshold value for xl interrupt event on generator.
    ///
    /// Data format is the same as output data raw: two's complement with
    /// 1LSb = 1.5mg.
    pub fn int_gen_threshold_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        IntThs::new().with_threshold(val).write(self)
    }
    /// Get User-defined threshold value for xl interrupt event on generator.
    ///
    /// Data format is the same as output data raw: two�s complement with
    /// 1LSb = 1.5mg.
    pub fn int_gen_threshold_get(&mut self) -> Result<u16, Error<B::Error>> {
        Ok(IntThs::read(self)?.threshold())
    }
    /// Set SPI Serial Interface Mode.
    ///
    /// If Sim::Spi4Wire enable SDO line on pin 7.
    pub fn spi_mode_set(&mut self, val: Sim) -> Result<(), Error<B::Error>> {
        let mut reg_val = CtrlReg3::read(self)?;
        reg_val.set_sim(val as u8);
        reg_val.write(self)?;

        Ok(())
    }

    /// Get SPI Serial Interface Mode.
    pub fn spi_mode_get(&mut self) -> Result<Sim, Error<B::Error>> {
        let reg_val = CtrlReg3::read(self)?;

        let val = Sim::try_from(reg_val.sim()).unwrap_or_default();
        Ok(val)
    }
}

pub fn from_lsb_to_mgauss(lsb: i16) -> f32 {
    lsb as f32 * 1.5
}

pub fn from_lsb_to_celsius(lsb: i16) -> f32 {
    lsb as f32 / 8.0 + 25.0
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum I2CAddress {
    I2cAdd = 0x1E,
}

pub const ID: u8 = 0x40;
