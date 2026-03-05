#![no_std]

use embedded_hal_async::i2c::I2c;
use embedded_hal_async::spi::SpiDevice;

const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL1_XL: u8 = 0x10;
const REG_CTRL3_C: u8 = 0x12;
const REG_STATUS_REG: u8 = 0x1E;
const REG_OUT_TEMP_L: u8 = 0x20;
const REG_OUTX_L_A: u8 = 0x28;

const WHO_AM_I_VALUE: u8 = 0x6B;
const CTRL1_ODR_MASK: u8 = 0b1111_0000;
const CTRL1_FS_MASK: u8 = 0b0000_1100;
const CTRL3_BDU: u8 = 1 << 6;
const CTRL3_IF_INC: u8 = 1 << 2;
const STATUS_XLDA: u8 = 1 << 0;
const STATUS_TDA: u8 = 1 << 2;

pub const ADDRESS_SA0_LOW: u8 = 0x6A;
pub const ADDRESS_SA0_HIGH: u8 = 0x6B;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error<E> {
    Bus(E),
    InvalidWhoAmI(u8),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OutputDataRate {
    PowerDown,
    Hz12_5,
    Hz26,
    Hz52,
    Hz104,
    Hz208,
    Hz416,
    Hz833,
}

impl OutputDataRate {
    const fn bits(self) -> u8 {
        match self {
            Self::PowerDown => 0b0000,
            Self::Hz12_5 => 0b0001,
            Self::Hz26 => 0b0010,
            Self::Hz52 => 0b0011,
            Self::Hz104 => 0b0100,
            Self::Hz208 => 0b0101,
            Self::Hz416 => 0b0110,
            Self::Hz833 => 0b0111,
        }
    }

    const fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            0b0000 => Some(Self::PowerDown),
            0b0001 => Some(Self::Hz12_5),
            0b0010 => Some(Self::Hz26),
            0b0011 => Some(Self::Hz52),
            0b0100 => Some(Self::Hz104),
            0b0101 => Some(Self::Hz208),
            0b0110 => Some(Self::Hz416),
            0b0111 => Some(Self::Hz833),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FullScale {
    G0_5,
    G3,
    G1,
    G2,
}

impl FullScale {
    const fn bits(self) -> u8 {
        match self {
            Self::G0_5 => 0b00,
            Self::G3 => 0b01,
            Self::G1 => 0b10,
            Self::G2 => 0b11,
        }
    }

    const fn from_bits(bits: u8) -> Self {
        match bits {
            0b00 => Self::G0_5,
            0b01 => Self::G3,
            0b10 => Self::G1,
            0b11 => Self::G2,
            _ => unreachable!(),
        }
    }

    const fn sensitivity_mg_per_lsb(self) -> f32 {
        match self {
            Self::G0_5 => 0.015,
            Self::G1 => 0.031,
            Self::G2 => 0.061,
            Self::G3 => 0.122,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    pub odr: OutputDataRate,
    pub fs: FullScale,
    pub lpf2_enable: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            odr: OutputDataRate::Hz104,
            fs: FullScale::G2,
            lpf2_enable: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RawAccel {
    pub x: i16,
    pub y: i16,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Accel {
    pub x_mg: f32,
    pub y_mg: f32,
}

pub struct Iis2Iclx<I2C> {
    i2c: I2C,
    address: u8,
    fs: FullScale,
}

impl<I2C, E> Iis2Iclx<I2C>
where
    I2C: I2c<Error = E>,
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            fs: FullScale::G2,
        }
    }

    pub fn release(self) -> I2C {
        self.i2c
    }

    pub async fn init(&mut self, config: Config) -> Result<(), Error<E>> {
        let who = self.read_who_am_i().await?;
        if who != WHO_AM_I_VALUE {
            return Err(Error::InvalidWhoAmI(who));
        }

        self.write_reg(REG_CTRL3_C, CTRL3_BDU | CTRL3_IF_INC).await?;

        let ctrl1 = (config.odr.bits() << 4)
            | (config.fs.bits() << 2)
            | ((config.lpf2_enable as u8) << 1);
        self.write_reg(REG_CTRL1_XL, ctrl1).await?;
        self.fs = config.fs;
        Ok(())
    }

    pub async fn read_who_am_i(&mut self) -> Result<u8, Error<E>> {
        self.read_reg(REG_WHO_AM_I).await
    }

    pub async fn read_output_data_rate(&mut self) -> Result<Option<OutputDataRate>, Error<E>> {
        let ctrl1 = self.read_reg(REG_CTRL1_XL).await?;
        Ok(OutputDataRate::from_bits((ctrl1 & CTRL1_ODR_MASK) >> 4))
    }

    pub async fn write_output_data_rate(&mut self, odr: OutputDataRate) -> Result<(), Error<E>> {
        let ctrl1 = self.read_reg(REG_CTRL1_XL).await?;
        let updated = (ctrl1 & !CTRL1_ODR_MASK) | (odr.bits() << 4);
        self.write_reg(REG_CTRL1_XL, updated).await
    }

    pub async fn read_full_scale(&mut self) -> Result<FullScale, Error<E>> {
        let ctrl1 = self.read_reg(REG_CTRL1_XL).await?;
        let fs = FullScale::from_bits((ctrl1 & CTRL1_FS_MASK) >> 2);
        self.fs = fs;
        Ok(fs)
    }

    pub async fn write_full_scale(&mut self, fs: FullScale) -> Result<(), Error<E>> {
        let ctrl1 = self.read_reg(REG_CTRL1_XL).await?;
        let updated = (ctrl1 & !CTRL1_FS_MASK) | (fs.bits() << 2);
        self.write_reg(REG_CTRL1_XL, updated).await?;
        self.fs = fs;
        Ok(())
    }

    pub async fn status(&mut self) -> Result<u8, Error<E>> {
        self.read_reg(REG_STATUS_REG).await
    }

    pub async fn accel_data_ready(&mut self) -> Result<bool, Error<E>> {
        Ok(self.status().await? & STATUS_XLDA != 0)
    }

    pub async fn temp_data_ready(&mut self) -> Result<bool, Error<E>> {
        Ok(self.status().await? & STATUS_TDA != 0)
    }

    pub async fn read_raw_accel(&mut self) -> Result<RawAccel, Error<E>> {
        let mut buf = [0u8; 4];
        self.read_regs(REG_OUTX_L_A, &mut buf).await?;
        Ok(RawAccel {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
        })
    }

    pub async fn read_accel_mg(&mut self) -> Result<Accel, Error<E>> {
        let raw = self.read_raw_accel().await?;
        let s = self.fs.sensitivity_mg_per_lsb();
        Ok(Accel {
            x_mg: raw.x as f32 * s,
            y_mg: raw.y as f32 * s,
        })
    }

    pub async fn read_raw_temperature(&mut self) -> Result<i16, Error<E>> {
        let mut buf = [0u8; 2];
        self.read_regs(REG_OUT_TEMP_L, &mut buf).await?;
        Ok(i16::from_le_bytes(buf))
    }

    pub async fn read_temperature_c(&mut self) -> Result<f32, Error<E>> {
        let raw = self.read_raw_temperature().await?;
        Ok(25.0 + (raw as f32 / 256.0))
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error<E>> {
        let mut out = [0u8; 1];
        self.i2c
            .write_read(self.address, &[reg], &mut out)
            .await
            .map_err(Error::Bus)?;
        Ok(out[0])
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &[reg, value])
            .await
            .map_err(Error::Bus)
    }

    async fn read_regs(&mut self, start_reg: u8, out: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c
            .write_read(self.address, &[start_reg], out)
            .await
            .map_err(Error::Bus)
    }
}

pub struct Iis2IclxSpi<SPI> {
    spi: SPI,
    fs: FullScale,
}

impl<SPI, E> Iis2IclxSpi<SPI>
where
    SPI: SpiDevice<u8, Error = E>,
{
    pub fn new(spi: SPI) -> Self {
        Self {
            spi,
            fs: FullScale::G2,
        }
    }

    pub fn release(self) -> SPI {
        self.spi
    }

    pub async fn init(&mut self, config: Config) -> Result<(), Error<E>> {
        let who = self.read_who_am_i().await?;
        if who != WHO_AM_I_VALUE {
            return Err(Error::InvalidWhoAmI(who));
        }

        self.write_reg(REG_CTRL3_C, CTRL3_BDU | CTRL3_IF_INC).await?;

        let ctrl1 = (config.odr.bits() << 4)
            | (config.fs.bits() << 2)
            | ((config.lpf2_enable as u8) << 1);
        self.write_reg(REG_CTRL1_XL, ctrl1).await?;
        self.fs = config.fs;
        Ok(())
    }

    pub async fn read_who_am_i(&mut self) -> Result<u8, Error<E>> {
        self.read_reg(REG_WHO_AM_I).await
    }

    pub async fn read_output_data_rate(&mut self) -> Result<Option<OutputDataRate>, Error<E>> {
        let ctrl1 = self.read_reg(REG_CTRL1_XL).await?;
        Ok(OutputDataRate::from_bits((ctrl1 & CTRL1_ODR_MASK) >> 4))
    }

    pub async fn write_output_data_rate(&mut self, odr: OutputDataRate) -> Result<(), Error<E>> {
        let ctrl1 = self.read_reg(REG_CTRL1_XL).await?;
        let updated = (ctrl1 & !CTRL1_ODR_MASK) | (odr.bits() << 4);
        self.write_reg(REG_CTRL1_XL, updated).await
    }

    pub async fn read_full_scale(&mut self) -> Result<FullScale, Error<E>> {
        let ctrl1 = self.read_reg(REG_CTRL1_XL).await?;
        let fs = FullScale::from_bits((ctrl1 & CTRL1_FS_MASK) >> 2);
        self.fs = fs;
        Ok(fs)
    }

    pub async fn write_full_scale(&mut self, fs: FullScale) -> Result<(), Error<E>> {
        let ctrl1 = self.read_reg(REG_CTRL1_XL).await?;
        let updated = (ctrl1 & !CTRL1_FS_MASK) | (fs.bits() << 2);
        self.write_reg(REG_CTRL1_XL, updated).await?;
        self.fs = fs;
        Ok(())
    }

    pub async fn status(&mut self) -> Result<u8, Error<E>> {
        self.read_reg(REG_STATUS_REG).await
    }

    pub async fn accel_data_ready(&mut self) -> Result<bool, Error<E>> {
        Ok(self.status().await? & STATUS_XLDA != 0)
    }

    pub async fn temp_data_ready(&mut self) -> Result<bool, Error<E>> {
        Ok(self.status().await? & STATUS_TDA != 0)
    }

    pub async fn read_raw_accel(&mut self) -> Result<RawAccel, Error<E>> {
        let mut buf = [0u8; 4];
        self.read_regs(REG_OUTX_L_A, &mut buf).await?;
        Ok(RawAccel {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
        })
    }

    pub async fn read_accel_mg(&mut self) -> Result<Accel, Error<E>> {
        let raw = self.read_raw_accel().await?;
        let s = self.fs.sensitivity_mg_per_lsb();
        Ok(Accel {
            x_mg: raw.x as f32 * s,
            y_mg: raw.y as f32 * s,
        })
    }

    pub async fn read_raw_temperature(&mut self) -> Result<i16, Error<E>> {
        let mut buf = [0u8; 2];
        self.read_regs(REG_OUT_TEMP_L, &mut buf).await?;
        Ok(i16::from_le_bytes(buf))
    }

    pub async fn read_temperature_c(&mut self) -> Result<f32, Error<E>> {
        let raw = self.read_raw_temperature().await?;
        Ok(25.0 + (raw as f32 / 256.0))
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error<E>> {
        let mut buf = [reg | 0x80, 0u8];
        self.spi.transfer_in_place(&mut buf).await.map_err(Error::Bus)?;
        Ok(buf[1])
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<E>> {
        let buf = [reg & 0x7F, value];
        self.spi.write(&buf).await.map_err(Error::Bus)
    }

    async fn read_regs(&mut self, start_reg: u8, out: &mut [u8]) -> Result<(), Error<E>> {
        // Enough for 1-byte command + up to 7 data bytes (current driver uses 2 and 4).
        assert!(out.len() <= 7);
        let mut buf = [0u8; 8];
        // Use only the SPI read bit (bit7). Address auto-increment is handled by IF_INC in CTRL3_C.
        buf[0] = start_reg | 0x80;
        self.spi
            .transfer_in_place(&mut buf[..out.len() + 1])
            .await
            .map_err(Error::Bus)?;
        out.copy_from_slice(&buf[1..out.len() + 1]);
        Ok(())
    }
}
