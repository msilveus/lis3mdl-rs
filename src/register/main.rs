use crate::Error;
use crate::Lis3mdl;
use bitfield_struct::bitfield;
use derive_more::TryFrom;
use st_mem_bank_macro::{named_register, register};
use st_mems_bus::BusOperation;

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum Reg {
    OffsetXRegL = 0x05,
    OffsetXRegH = 0x06,
    OffsetYRegL = 0x07,
    OffsetYRegH = 0x08,
    OffsetZRegL = 0x09,
    OffsetZRegH = 0x0A,
    WhoAmI = 0x0F,
    CtrlReg1 = 0x20,
    CtrlReg2 = 0x21,
    CtrlReg3 = 0x22,
    CtrlReg4 = 0x23,
    CtrlReg5 = 0x24,
    StatusReg = 0x27,
    OutxLReg = 0x28,
    OutxHReg = 0x29,
    OutyLReg = 0x2A,
    OutyHReg = 0x2B,
    OutzLReg = 0x2C,
    OutzHReg = 0x2D,
    TempOutLReg = 0x2E,
    TempOutHReg = 0x2F,
    IntCfgReg = 0x30,
    IntSourceReg = 0x31,
    IntThsLReg = 0x32,
    IntThsHReg = 0x33,
}

/// Offset XYZ (0x05 - 0x0A)
///
/// Magnetic user offset for hard-iron to compensate environmental
/// effects.
/// 1LSB = 1.5mg
#[named_register(address = Reg::OffsetXRegL, access_type = Lis3mdl, generics = 1)]
pub struct OffsetXYZ {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// WHO_AM_I (0x0F)
///
/// The identification register is used to identify the device
#[register(address = Reg::WhoAmI, access_type = Lis3mdl, generics = 1)]
pub struct WhoAmI(pub u8);

/// Ctrl_Reg1 (0x20)
///
/// Configuration register 1 (R/W)
/// Controls output data rate, power mode, reboot, soft reset, and temperature compensation.
#[register(address = Reg::CtrlReg1, access_type = Lis3mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CtrlReg1 {
    /// Self-Test (1 bits)
    /// Enables self-test. Default value: 0
    /// (0: self-test disabled; 1: self-test enabled)
    #[bits(1)]
    pub st: u8,

    /// FAST_ODR enables data rates higher than 80 Hz. Default value: 0
    /// (0: FAST_ODR disabled; 1: FAST_ODR enabled)
    /// 1000 ODR when om is set for LP mode
    /// 560 ODR when om is set for MP mode
    /// 300 ODR when om is set for HP mode
    /// 155 ODR when om is set for UHP mode
    #[bits(1)]
    pub fast_odr: u8,

    /// Output data rate selection. Default value: 100
    /// Connected enum: Odr
    #[bits(3)]
    pub odr: u8,

    /// X and Y axes operative mode selection. Default value: 00
    /// Connected enum: Om
    #[bits(2)]
    pub om: u8,

    /// Enables temperature sensor. Default value: 0
    /// (0: temperature sensor disabled; 1: temperature sensor enabled)
    #[bits(1)]
    pub temp_en: u8,
}

/// Ctrl_Reg2 (0x21)
///
/// Configuration register B (R/W)
/// Controls offset cancellation, interrupt on data off, set pulse frequency, and low-pass filter.
#[register(address = Reg::CtrlReg2, access_type = Lis3mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CtrlReg2 {
    /// Reserved bits (2 bits, read-only)
    #[bits(2, access = RO)]
    pub not_used_01: u8,

    /// Configuration registers and user register reset function.
    /// (0: default value; 1: reset operation)
    #[bits(1)]
    pub soft_reset: u8,

    /// Reboots memory content. Default value: 0
    /// (0: normal mode; 1: reboot memory content)
    #[bits(1)]
    pub reboot: u8,

    /// Reserved bits (2 bits, read-only)
    #[bits(1, access = RO)]
    pub not_used_4: u8,

    /// Full-scale configuration (refer to Table 24). Default value: 00
    /// Connected enum: FullScale
    #[bits(2)]
    pub fs: u8,

    /// Reserved bits (3 bits, read-only)
    #[bits(1, access = RO)]
    pub not_used_7: u8,
}

/// Ctrl_Reg3 (0x22)
///
/// Configuration register C (R/W)
/// Controls interrupt pin behavior, interface selection, byte order, SPI mode, self-test, and data-ready pin.
#[register(address = Reg::CtrlReg3, access_type = Lis3mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CtrlReg3 {
    /// Operating mode selection (refer to Table 27). Default value: 11
    /// Connected enum: Md
    #[bits(2)]
    pub md: u8,

    /// SPI serial interface mode selection. Default value: 0
    /// (0: 4-wire interface; 1: 3-wire interface)
    /// Enable 4-wire SPI interface (disables interrupt and data-ready signaling) (1 bit)
    /// Connected enum: Sim
    #[bits(1)]
    pub sim: u8,

    /// Reserved bits (3 bits, read-only)
    #[bits(2, access = RO)]
    pub not_used_34: u8,

    /// Low-power mode configuration. Default value: 0
    /// If this bit is 1, DO[2:0] is set to 0.625 Hz and the system performs, for each channel, the minimum number of
    /// averages. Once the bit is set to 0, the magnetic data rate is configured by the DO bits in the CTRL_REG1 (20h)
    /// register.
    #[bits(1)]
    pub lp: u8,

    /// Reserved bits (3 bits, read-only)
    #[bits(2, access = RO)]
    pub not_used_67: u8,
}

/// Ctrl_Reg4 (0x23)
///
/// Configuration register Ctrl_Reg4 (R/W)
/// Controls interrupt pin behavior, interface selection, byte order, SPI mode, self-test, and data-ready pin.
#[register(address = Reg::CtrlReg4, access_type = Lis3mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CtrlReg4 {
    /// Reserved bits (1 bits, read-only)
    #[bits(1, access = RO)]
    pub not_used_0: u8,

    /// Big/little endian data selection. Default value: 0
    /// (0: data LSb at lower address; 1: data MSb at lower address)
    #[bits(1)]
    pub ble: u8,

    /// Z-axis operative mode selection (refer to Table 30). Default value: 00
    /// Connected enum: Om
    #[bits(2)]
    pub omz: u8,

    /// Reserved bits (2 bits, read-only)
    #[bits(4, access = RO)]
    pub not_used_4567: u8,
}

/// Ctrl_Reg5 (0x24)
///
/// Configuration register Ctrl_Reg5 (R/W)
/// Controls interrupt pin behavior, interface selection, byte order, SPI mode, self-test, and data-ready pin.
#[register(address = Reg::CtrlReg5, access_type = Lis3mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CtrlReg5 {
    /// Reserved bits (1 bits, read-only)
    #[bits(6, access = RO)]
    pub not_used_012345: u8,

    /// Block data update for magnetic data. Default value: 0
    /// (0: continuous update;
    /// 1: output registers not updated until MSb and LSb have been read)
    #[bits(1)]
    pub bdu: u8,

    /// FAST READ allows reading the high part of DATA OUT only in order to increase reading efficiency. Default value: 0
    /// (0: FAST_READ disabled; 1: FAST_READ enabled)
    #[bits(1)]
    pub fast_read: u8,
}

/// STATUS_REG (0x27)
///
/// Status register (R)
/// Indicates data availability and overrun status for each axis.
#[register(address = Reg::StatusReg, access_type = Lis3mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct StatusReg {
    /// X-axis new data available (1 bit)
    #[bits(1)]
    pub xda: u8,
    /// Y-axis new data available (1 bit)
    #[bits(1)]
    pub yda: u8,
    /// Z-axis new data available (1 bit)
    #[bits(1)]
    pub zda: u8,
    /// X-, Y-, and Z-axis new data available (1 bit)
    #[bits(1)]
    pub zyxda: u8,
    /// X-axis data overrun (1 bit)
    #[bits(1)]
    pub _xor: u8,
    /// Y-axis data overrun (1 bit)
    #[bits(1)]
    pub yor: u8,
    /// Z-axis data overrun (1 bit)
    #[bits(1)]
    pub zor: u8,
    /// X-, Y-, and Z-axis data overrun (1 bit)
    #[bits(1)]
    pub zyxor: u8,
}

/// Int_Cfg (0x63)
///
/// Interrupt control register (R/W)
/// Enables and configures interrupt recognition.
#[register(address = Reg::IntCfgReg, access_type = Lis3mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct IntCfgReg {
    /// Interrupt enable (1 bit)
    /// Enables generation of interrupt (INT bit in INT_SOURCE_REG).
    #[bits(1)]
    pub ien: u8,

    /// Latch interrupt request. Default value: 0
    /// (0: interrupt request latched; 1: interrupt request not latched)
    /// Once latched, the INT pin remains in the same state until INT_SRC (31h) is read.
    #[bits(1)]
    pub lir: u8,

    /// Interrupt active configuration on INT. Default value: 0
    /// (0: low; 1: high)
    #[bits(1)]
    pub iea: u8,

    /// Reserved bits (1 bits, read-only)
    #[bits(1, access = RO)]
    pub not_used_3: u8,

    /// Reserved bits (1 bits, read-only)
    #[bits(1, access = RO)]
    pub not_used_4: u8,

    /// Z-axis interrupt enable (1 bit)
    /// (0: disable interrupt request; 1: enable interrupt request)
    #[bits(1)]
    pub zien: u8,

    /// Y-axis interrupt enable (1 bit)
    /// (0: disable interrupt request; 1: enable interrupt request)
    #[bits(1)]
    pub yien: u8,

    /// X-axis interrupt enable (1 bit)
    /// (0: disable interrupt request; 1: enable interrupt request)
    #[bits(1)]
    pub xien: u8,
}

/// INT_SOURCE_REG (0x64)
///
/// Interrupt source register (R)
/// Indicates interrupt event status and axis threshold crossing.
#[register(address = Reg::IntSourceReg, access_type = Lis3mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct IntSourceReg {
    /// Interrupt event occurred (1 bit)
    #[bits(1)]
    pub _int: u8,

    /// Magnetic ROI flag (1 bit)
    /// Always enabled; reset by reading this register.
    #[bits(1)]
    pub mroi: u8,

    /// Negative threshold exceeded on Z-axis (1 bit)
    #[bits(1)]
    pub n_th_s_z: u8,

    /// Negative threshold exceeded on Y-axis (1 bit)
    #[bits(1)]
    pub n_th_s_y: u8,

    /// Negative threshold exceeded on X-axis (1 bit)
    #[bits(1)]
    pub n_th_s_x: u8,

    /// Positive threshold exceeded on Z-axis (1 bit)
    #[bits(1)]
    pub p_th_s_z: u8,

    /// Positive threshold exceeded on Y-axis (1 bit)
    #[bits(1)]
    pub p_th_s_y: u8,

    /// Positive threshold exceeded on X-axis (1 bit)
    #[bits(1)]
    pub p_th_s_x: u8,
}

/// INT_THS_L_REG - INT_THS_H_REG (0x65 - 0x66)
///
/// These registers set the threshold value for the output to generate the interrupt (INT bit in INT_SOURCE_REG(64h)).
/// This threshold is common to all three (axes) output values and is unsigned unipolar. The threshold value is correlated
/// to the current gain and it is unsigned because the threshold is considered as an absolute value, but crossing the
/// threshold is detected for both the positive and negative sides.
#[register(address = Reg::IntThsLReg, access_type = Lis3mdl, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct IntThs {
    #[bits(16)]
    pub threshold: u16,
}

/// OutXYZ (0x68 - 0x6D)
///
/// The output data represents the raw magnetic data only if
/// relative OFFSET_axis_REG is equal to zero, otherwise hard-iron
/// calibration is included.
#[named_register(address = Reg::OutxLReg, access_type = Lis3mdl, generics = 1)]
pub struct OutXYZ {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// TempOut (0x6E - 0x6F)
///
/// Temperature register of the sensor
#[register(address = Reg::TempOutLReg, access_type = Lis3mdl, generics = 1)]
pub struct TempOut(pub i16);

/// Mode of operation
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default)]
pub enum Md {
    /// Continuous mode (continuous measurement, data-ready signal generated)
    ContinuousMode = 0,
    /// Single measurement mode (performs one measurement, sets DRDY high, then idle)
    SingleTrigger = 1,
    /// Idle mode (I2C and SPI active): bits 10 or 11
    #[default]
    PowerDown = 3,
}

impl TryFrom<u8> for Md {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        let md = match value {
            0 => Md::ContinuousMode,
            1 => Md::SingleTrigger,
            _ => Md::PowerDown,
        };

        Ok(md)
    }
}

/// Output data rate configuration for the sensor.
///
/// Determines the frequency at which data is updated and made available.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Odr {
    /// Output data rate at 0.625 Hz (default)
    Hz0_625 = 0,
    /// Output data rate at 1.25 Hz
    Hz1_25 = 1,
    /// Output data rate at 2.5 Hz
    Hz2_5 = 2,
    /// Output data rate at 5 Hz
    Hz5 = 3,
    /// Output data rate at 10 Hz
    #[default]
    Hz10 = 4,
    /// Output data rate at 20 Hz
    Hz20 = 5,
    /// Output data rate at 40 Hz
    Hz40 = 6,
    /// Output data rate at 80 Hz
    Hz80 = 7,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FullScale {
    /// +-2 gauss (default)
    #[default]
    G2 = 0,
    /// +-4 gauss
    G4 = 1,
    /// +-8 gauss
    G8 = 2,
    /// +-16 gauss
    G16 = 3,
}
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Om {
    /// Low-power mode (LP) (default)
    #[default]
    LowPowerMode = 0,
    /// Medium-performance mode (MP)
    MedPerformanceMode = 1,
    /// High-performance mode (HP)
    HiPerformanceMode = 2,
    /// Ultrahigh-performance mode (UHP)
    UltraHiPerformanceMode = 3,
}

/// Power mode
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum PowerMode {
    /// High-resolution mode
    #[default]
    HighResolution = 0,
    /// Low-power mode enabled
    LowPower = 1,
}

/// Digital low pass filter
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum LowPassFilter {
    /// Filter disabled (bandwidth = ODR/2)
    #[default]
    OdrDiv2 = 0,
    /// Filter enabled (bandwidth = ODR/4)
    OdrDiv4 = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum SetRst {
    /// Set pulse released every 64 ODR cycles (default)
    #[default]
    SetSensOdrDiv63 = 0,
    /// Set pulse released only at power-on after power-down condition
    SensOffCancEveryOdr = 1,
    /// Set pulse released only at power-on after power-down condition
    SetSensOnlyAtPowerOn = 2,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Ble {
    /// LSB at low address (default byte order)
    #[default]
    LsbAtLowAdd = 0,
    /// MSB at low address (byte order inverted)
    MsbAtLowAdd = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum IntOnDataoff {
    /// Interrupt recognition checks data before hard-iron correction
    #[default]
    CheckBefore = 0,
    /// Interrupt recognition checks data after hard-iron correction
    CheckAfter = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Sim {
    /// SPI 4-wire interface enabled (disables interrupt and data-ready signaling)
    #[default]
    Spi4Wire = 0,
    /// SPI 3-wire interface enabled
    Spi3Wire = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum I2cSwitch {
    /// I2C interface enabled (default)
    #[default]
    I2cEnable = 0,
    /// I2C interface disabled; only SPI interface can be used
    I2cDisable = 1,
}
