//! Persistent storage for the on-chip IMU (BMI323) bias calibration.
//!
//! The BMI323 corrects gyro/accel bias internally via its data-path (DP) offset
//! registers, but those registers are volatile and lost on every power-cycle/soft
//! reset (the chip has no user NVM). To avoid re-running the calibration on every
//! boot we persist the applied offsets to a dedicated flash sector on the
//! STM32H723 and reload + re-apply them at startup.
//!
//! The blob is stored in the last 128 KiB flash sector (offset [`CAL_FLASH_OFFSET`]),
//! which is reserved (kept out of the firmware image) in `memory.x`. It is one
//! flash word (32 bytes, the H7 write granularity) and is validated on read with a
//! magic word, a format version, the firmware hash (so a firmware change that could
//! alter the accel range/config invalidates stale calibration), and a CRC32.

use ateam_lib_stm32::drivers::imu::bmi323::{AccelDpOffset, GyroDpOffsetGain};
use embassy_stm32::flash::{Blocking, Flash};

use crate::git_version::FIRMWARE_HASH;

/// Magic identifying a valid calibration blob ("IMUC").
const CAL_MAGIC: u32 = 0x494D_5543;
/// On-disk format version. Bump when the blob layout changes.
const CAL_VERSION: u32 = 1;
/// Offset of the calibration sector from the start of flash. This is the last
/// 128 KiB sector of the 1 MiB device (0x080E_0000), reserved in `memory.x`.
const CAL_FLASH_OFFSET: u32 = 0x000E_0000;
/// The whole sector is erased when writing (the H7 minimum erase unit is 128 KiB).
const CAL_SECTOR_LEN: u32 = 128 * 1024;
/// Serialized blob length. Must be a multiple of the 32-byte H7 flash write word.
const CAL_BLOB_LEN: usize = 32;

/// The IMU bias calibration applied to the on-chip data-path registers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ImuCalibration {
    pub gyro: GyroDpOffsetGain,
    pub accel: AccelDpOffset,
}

impl ImuCalibration {
    /// Serializes the calibration into a fixed-size, CRC-protected blob.
    fn serialize(&self) -> [u8; CAL_BLOB_LEN] {
        let mut buf = [0u8; CAL_BLOB_LEN];
        buf[0..4].copy_from_slice(&CAL_MAGIC.to_le_bytes());
        buf[4..8].copy_from_slice(&CAL_VERSION.to_le_bytes());
        buf[8..12].copy_from_slice(&FIRMWARE_HASH);
        buf[12..14].copy_from_slice(&self.gyro.off_x.to_le_bytes());
        buf[14..16].copy_from_slice(&self.gyro.off_y.to_le_bytes());
        buf[16..18].copy_from_slice(&self.gyro.off_z.to_le_bytes());
        buf[18] = self.gyro.dgain_x;
        buf[19] = self.gyro.dgain_y;
        buf[20] = self.gyro.dgain_z;
        buf[21] = 0; // reserved / padding
        buf[22..24].copy_from_slice(&self.accel.off_x.to_le_bytes());
        buf[24..26].copy_from_slice(&self.accel.off_y.to_le_bytes());
        buf[26..28].copy_from_slice(&self.accel.off_z.to_le_bytes());
        let crc = crc32(&buf[0..28]);
        buf[28..32].copy_from_slice(&crc.to_le_bytes());
        buf
    }

    /// Parses and validates a blob previously written by [`Self::serialize`].
    /// Returns `None` if the magic, version, firmware hash, or CRC do not match.
    fn deserialize(buf: &[u8; CAL_BLOB_LEN]) -> Option<Self> {
        if u32::from_le_bytes(buf[0..4].try_into().unwrap()) != CAL_MAGIC {
            return None;
        }
        if u32::from_le_bytes(buf[4..8].try_into().unwrap()) != CAL_VERSION {
            return None;
        }
        if buf[8..12] != FIRMWARE_HASH {
            return None;
        }
        let stored_crc = u32::from_le_bytes(buf[28..32].try_into().unwrap());
        if crc32(&buf[0..28]) != stored_crc {
            return None;
        }

        Some(Self {
            gyro: GyroDpOffsetGain {
                off_x: i16::from_le_bytes(buf[12..14].try_into().unwrap()),
                off_y: i16::from_le_bytes(buf[14..16].try_into().unwrap()),
                off_z: i16::from_le_bytes(buf[16..18].try_into().unwrap()),
                dgain_x: buf[18],
                dgain_y: buf[19],
                dgain_z: buf[20],
            },
            accel: AccelDpOffset {
                off_x: i16::from_le_bytes(buf[22..24].try_into().unwrap()),
                off_y: i16::from_le_bytes(buf[24..26].try_into().unwrap()),
                off_z: i16::from_le_bytes(buf[26..28].try_into().unwrap()),
            },
        })
    }
}

/// Reads and validates the stored IMU calibration from flash. Returns `None` if no
/// valid blob is present (blank flash, version/firmware mismatch, or CRC failure).
pub fn load_calibration(flash: &mut Flash<'static, Blocking>) -> Option<ImuCalibration> {
    let mut buf = [0u8; CAL_BLOB_LEN];
    if flash.blocking_read(CAL_FLASH_OFFSET, &mut buf).is_err() {
        defmt::warn!("IMU calibration flash read failed");
        return None;
    }
    ImuCalibration::deserialize(&buf)
}

/// Erases the calibration sector and writes the given calibration to flash.
/// Returns `Err` if the erase or write fails.
pub fn store_calibration(
    flash: &mut Flash<'static, Blocking>,
    cal: &ImuCalibration,
) -> Result<(), ()> {
    let buf = cal.serialize();
    // Run the erase + program with interrupts disabled. This flash is single-bank, so
    // while a sector is being erased/programmed any concurrent access to the bank
    // (e.g. an interrupt handler's instruction fetch) can abort the operation
    // (FLASH_SR.OPERR). Masking interrupts for the few-ms operation avoids that; the
    // IMU is inop (wheels locked out) during calibration so the brief stall is safe.
    let res = critical_section::with(|_| {
        flash.blocking_erase(CAL_FLASH_OFFSET, CAL_FLASH_OFFSET + CAL_SECTOR_LEN)?;
        flash.blocking_write(CAL_FLASH_OFFSET, &buf)
    });
    if let Err(e) = res {
        defmt::error!("IMU calibration flash store failed: {}", e);
        return Err(());
    }
    Ok(())
}

/// Erases the calibration sector, deleting any stored IMU calibration. After this
/// the blob reads as blank (magic mismatch), so [`load_calibration`] returns `None`
/// and the IMU is treated as uncalibrated.
pub fn erase_calibration(flash: &mut Flash<'static, Blocking>) -> Result<(), ()> {
    // See `store_calibration`: mask interrupts so a concurrent flash access cannot
    // abort the erase on this single-bank device.
    let res = critical_section::with(|_| {
        flash.blocking_erase(CAL_FLASH_OFFSET, CAL_FLASH_OFFSET + CAL_SECTOR_LEN)
    });
    if let Err(e) = res {
        defmt::error!("IMU calibration flash erase failed: {}", e);
        return Err(());
    }
    Ok(())
}

/// Standard CRC-32 (IEEE 802.3, reflected, poly 0xEDB88520). Computed once at boot
/// and once per (re)calibration, so a small bitwise implementation is fine.
fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}
