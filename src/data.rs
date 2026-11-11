//! The data path: the FIFO methods, [`Lsm6dsk320x::interrupt_status`] and the counters.
//!
//! Everything here is integer code with no allocation; the only division is by the constant 7
//! (the word length), on `usize`.

use embedded_hal_async::spi::SpiDevice;

use crate::config::FifoMode;
use crate::{Error, Lsm6dsk320x, reg};

/// Bytes in one FIFO word: a tag byte and six data bytes (6.5.8, p46).
const WORD: usize = reg::FIFO_WORD_LEN;

/// The flags of FIFO_STATUS2 (9.26, Tables 80-81, p75).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct FifoFlags {
    /// FIFO_WTM_IA: the unread words are at or above the watermark.
    pub watermark: bool,
    /// FIFO_FULL_IA: the FIFO will be full at the next ODR.
    pub full: bool,
    /// FIFO_OVR_IA: overrun status, as the device reports it now. A state, confirmed on the board,
    /// 2026-11-10: every status read sees it while the FIFO stays full.
    pub overrun: bool,
    /// FIFO_OVR_LATCHED: an overrun happened since this register was last read. The device clears
    /// it when FIFO_STATUS2 is read (p75), which every method that reports it has just done, so
    /// it is reported exactly once and counted in [`Stats::fifo_overruns`].
    pub overrun_latched: bool,
}

impl FifoFlags {
    const fn from_status2(b: u8) -> Self {
        Self {
            watermark: b & reg::FIFO_STATUS2_FIFO_WTM_IA != 0,
            full: b & reg::FIFO_STATUS2_FIFO_FULL_IA != 0,
            overrun: b & reg::FIFO_STATUS2_FIFO_OVR_IA != 0,
            overrun_latched: b & reg::FIFO_STATUS2_FIFO_OVR_LATCHED != 0,
        }
    }

    /// True when either overrun flag is set.
    pub const fn any_overrun(&self) -> bool {
        self.overrun || self.overrun_latched
    }
}

/// The result of [`Lsm6dsk320x::fifo_status`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct FifoStatus {
    /// Unread words in the FIFO: DIFF_FIFO\[8:0\] (9.25 and 9.26, p75).
    pub words: u16,
    /// The flags read with the count.
    pub flags: FifoFlags,
}

impl FifoStatus {
    fn from_bytes(status1: u8, status2: u8) -> Self {
        Self {
            words: u16::from(status2 & reg::FIFO_STATUS2_DIFF_FIFO_8) << 8 | u16::from(status1),
            flags: FifoFlags::from_status2(status2),
        }
    }
}

/// The result of [`Lsm6dsk320x::fifo_read`] and [`Lsm6dsk320x::fifo_read_exact`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct FifoRead {
    /// Bytes written to the start of the caller's buffer: `7 * words`.
    pub bytes: usize,
    /// Whole words read.
    pub words: usize,
    /// Words still unread in the device after this call, as far as the status read knows (words
    /// that arrived since are not counted). `None` after [`Lsm6dsk320x::fifo_read_exact`], which
    /// reads no status.
    pub unread: Option<u16>,
    /// The flags the status read carried. `None` after [`Lsm6dsk320x::fifo_read_exact`], which
    /// reads no status: `None` means "not read", while `Some` with every flag false means "read,
    /// and there was no overrun".
    pub flags: Option<FifoFlags>,
}

/// The sources [`Config`](crate::Config) can route to the interrupt pins, as the device reports
/// them. See [`Lsm6dsk320x::interrupt_status`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct InterruptStatus {
    /// STATUS_REG XLDA: new low-g accelerometer data (9.28, p77).
    pub accel_data_ready: bool,
    /// STATUS_REG GDA: new gyroscope data.
    pub gyro_data_ready: bool,
    /// STATUS_REG XLHGDA: new high-g accelerometer data.
    pub high_g_data_ready: bool,
    /// STATUS_REG TIMESTAMP_ENDCOUNT: the timestamp overflows within 5.6 ms.
    pub timestamp_overflow: bool,
    /// The FIFO flags of FIFO_STATUS2 (9.26, p75).
    pub fifo: FifoFlags,
    /// HG_WAKE_UP_SRC HG_WU_IA: high-g wake-up event (9.51, p90). False when the applied
    /// configuration has no high-g events.
    pub high_g_wake_up: bool,
    /// HG_WAKE_UP_SRC HG_SHOCK_CHANGE_IA: high-g shock state change. False without high-g events.
    pub high_g_shock: bool,
}

/// Counters that nothing clears silently (contract C4). They saturate at `u32::MAX`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Stats {
    /// Calls of [`Lsm6dsk320x::fifo_read`], [`Lsm6dsk320x::fifo_status`] and
    /// [`Lsm6dsk320x::interrupt_status`] whose FIFO status showed an overrun (FIFO_OVR_IA or
    /// FIFO_OVR_LATCHED). It counts the CALLS that saw the overrun state, one per call however many
    /// words were lost. FIFO_OVR_IA is a state (confirmed on the board, 2026-11-10: every status
    /// read sees it while the FIFO stays full), so a FIFO left full is counted again by every
    /// status read.
    pub fifo_overruns: u32,
    /// FIFO words read by completed [`Lsm6dsk320x::fifo_read`] and
    /// [`Lsm6dsk320x::fifo_read_exact`] calls.
    pub fifo_words: u32,
    /// Every bus error the driver returns, from any method: the raw register methods
    /// ([`Lsm6dsk320x::read_register`], [`Lsm6dsk320x::write_register`],
    /// [`Lsm6dsk320x::read_registers`]) are included.
    pub bus_errors: u32,
}

impl Stats {
    /// All counters at 0.
    pub const fn new() -> Self {
        Self {
            fifo_overruns: 0,
            fifo_words: 0,
            bus_errors: 0,
        }
    }
}

impl<SPI: SpiDevice> Lsm6dsk320x<SPI> {
    /// The counters. They are never cleared by the driver, not even by [`init`](Self::init).
    pub const fn stats(&self) -> &Stats {
        &self.stats
    }

    /// Test-only access to the counters, to drive one to its maximum.
    #[cfg(test)]
    pub(crate) fn stats_mut(&mut self) -> &mut Stats {
        &mut self.stats
    }

    /// What the last completed [`init`](Self::init) or [`configure`](Self::configure) computed;
    /// `None` while the driver is not configured.
    pub fn info(&self) -> Option<crate::Info> {
        self.applied.as_ref().map(|a| a.info)
    }

    fn fifo_ready(&self) -> Result<(), Error<SPI::Error>> {
        match self.applied.as_ref() {
            None => Err(Error::NotConfigured),
            Some(a) if a.config.fifo.mode == FifoMode::Bypass => Err(Error::FifoOff),
            Some(_) => Ok(()),
        }
    }

    fn note_flags(&mut self, flags: FifoFlags) {
        if flags.any_overrun() {
            self.stats.fifo_overruns = self.stats.fifo_overruns.saturating_add(1);
        }
    }

    /// FIFO_STATUS1 and FIFO_STATUS2 in one transaction. The overrun, if any, is counted before
    /// the next await: reading FIFO_STATUS2 has just cleared the latched flag (p75).
    async fn read_fifo_status(&mut self) -> Result<FifoStatus, Error<SPI::Error>> {
        let mut b = [0u8; 2];
        self.read_registers(reg::FIFO_STATUS1, &mut b).await?;
        let [status1, status2] = b;
        let status = FifoStatus::from_bytes(status1, status2);
        self.note_flags(status.flags);
        Ok(status)
    }

    /// One burst of `words` whole words from FIFO_DATA_OUT_TAG into the start of `buf`.
    ///
    /// One burst over many words: IF_INC makes the address advance at every byte of a multiple
    /// byte read (CTRL3, 9.16, p67; SPI, p25-26), and a word is the seven registers 78h to 7Eh
    /// (6.5.8, p46; Table 26, p55). DS15060 does not say in words that the address returns from
    /// 7Eh to 78h inside a burst; confirmed on the board, 2026-11-10: it does, and bursts of 208
    /// and 256 words decoded exactly like word-by-word reads.
    async fn read_words(&mut self, buf: &mut [u8], words: u16) -> Result<(), Error<SPI::Error>> {
        let len = usize::from(words) * WORD;
        let dst = buf.get_mut(..len).ok_or(Error::BufferTooSmall)?;
        self.read_registers(reg::FIFO_DATA_OUT_TAG, dst).await?;
        self.stats.fifo_words = self.stats.fifo_words.saturating_add(u32::from(words));
        Ok(())
    }

    /// Reads the FIFO status, then every unread word that fits in `buf`, whole words only.
    ///
    /// Transactions: 2 (`[Write(0x9B), Read(2)]` for FIFO_STATUS1 and FIFO_STATUS2, then
    /// `[Write(0xF8), Read(7 * n)]` from FIFO_DATA_OUT_TAG), or 1 when there is nothing to read
    /// or no room for a word after the status. `n` is the smaller of the unread words and
    /// `buf.len() / 7`. A partial word is never read: bytes of `buf` past `7 * n` are not
    /// touched, and words that do not fit stay in the device ([`FifoRead::unread`]). Decode the
    /// bytes with [`fifo_frames`](crate::fifo_frames).
    ///
    /// Errors, all before any bus access: [`Error::NotConfigured`], [`Error::FifoOff`] when the
    /// applied configuration has FIFO mode bypass, [`Error::BufferTooSmall`] when `buf` is shorter
    /// than one word.
    ///
    /// Loss: a word the device has shifted out is gone from the device. If the second transaction
    /// fails or its future is dropped after the device started sending, the words it had sent
    /// are lost to the caller (and not counted in [`Stats::fifo_words`]); call again for the
    /// rest. A transaction that did not start takes no word. An overrun the status showed is
    /// counted in [`Stats::fifo_overruns`] even when the second transaction then fails, because
    /// the status read has cleared the latched flag.
    pub async fn fifo_read(&mut self, buf: &mut [u8]) -> Result<FifoRead, Error<SPI::Error>> {
        self.fifo_ready()?;
        if buf.len() < WORD {
            return Err(Error::BufferTooSmall);
        }
        let status = self.read_fifo_status().await?;
        let fit = u16::try_from(buf.len() / WORD).unwrap_or(u16::MAX);
        let n = status.words.min(fit);
        if n > 0 {
            self.read_words(buf, n).await?;
        }
        Ok(FifoRead {
            bytes: usize::from(n) * WORD,
            words: usize::from(n),
            unread: Some(status.words - n),
            flags: Some(status.flags),
        })
    }

    /// Reads exactly `words` whole words into the start of `buf`: one transaction,
    /// `[Write(0xF8), Read(7 * words)]`, no status read. For a watermark interrupt, where the
    /// application knows that at least the watermark's worth of words is waiting.
    ///
    /// Errors, all before any bus access: [`Error::NotConfigured`], [`Error::FifoOff`],
    /// [`Error::ZeroWords`] for `words == 0`, [`Error::BufferTooSmall`] when `7 * words` bytes do
    /// not fit in `buf`. If fewer than `words` words are waiting the device sends what it has
    /// followed by its empty-FIFO bytes, which [`fifo_frames`](crate::fifo_frames) shows as
    /// unknown words (tag 0); the caller owns that precondition.
    ///
    /// The result has `unread == None` and `flags == None`: the status was not read. The loss
    /// rules of [`fifo_read`](Self::fifo_read) apply to the one transaction. Overruns are not
    /// counted, because no status is read; use [`fifo_status`](Self::fifo_status) for that.
    pub async fn fifo_read_exact(
        &mut self,
        buf: &mut [u8],
        words: usize,
    ) -> Result<FifoRead, Error<SPI::Error>> {
        self.fifo_ready()?;
        if words == 0 {
            return Err(Error::ZeroWords);
        }
        let len = words.checked_mul(WORD).ok_or(Error::BufferTooSmall)?;
        if len > buf.len() {
            return Err(Error::BufferTooSmall);
        }
        let n = u16::try_from(words).map_err(|_| Error::BufferTooSmall)?;
        self.read_words(buf, n).await?;
        Ok(FifoRead {
            bytes: len,
            words,
            unread: None,
            flags: None,
        })
    }

    /// The unread word count and the flags, in one transaction:
    /// `[Write(0x9B), Read(2)]` (FIFO_STATUS1, FIFO_STATUS2). Reading FIFO_STATUS2 clears
    /// FIFO_OVR_LATCHED (p75); it is reported here and counted in [`Stats::fifo_overruns`].
    ///
    /// [`Error::NotConfigured`] and [`Error::FifoOff`] before any bus access.
    pub async fn fifo_status(&mut self) -> Result<FifoStatus, Error<SPI::Error>> {
        self.fifo_ready()?;
        self.read_fifo_status().await
    }

    /// Empties the FIFO and puts it back in the mode the applied [`Config`](crate::Config) asked
    /// for: two writes of FIFO_CTRL4 (9.8, p61), first the applied image with FIFO_MODE = 000
    /// (bypass: "Bypass mode is also used to reset the FIFO", 6.5.1, p45; "To reset FIFO content,
    /// bypass mode should be selected ... it is possible to restart FIFO mode by writing
    /// FIFO_MODE", 6.5.2, p45), then the applied image. Words not yet read are discarded and not
    /// counted anywhere.
    ///
    /// Confirmed on the board, 2026-11-10: bypass, then the mode again, empties the FIFO and clears
    /// the flags.
    ///
    /// Transactions: 2 writes. If the second does not happen (bus error, dropped future) the FIFO
    /// stays in bypass, collecting nothing, while the driver is still configured:
    /// [`verify`](Self::verify) reports it as a FIFO_CTRL4 mismatch, and calling this again puts
    /// it right. [`Error::NotConfigured`] and [`Error::FifoOff`] before any bus access.
    pub async fn fifo_flush(&mut self) -> Result<(), Error<SPI::Error>> {
        self.fifo_ready()?;
        let image = match self.applied.as_ref() {
            Some(a) => a.config.fifo_ctrl4(),
            None => return Err(Error::NotConfigured),
        };
        self.write_register(reg::FIFO_CTRL4, image & !reg::FIFO_CTRL4_FIFO_MODE_MASK)
            .await?;
        self.write_register(reg::FIFO_CTRL4, image).await
    }

    /// The interrupt sources [`Config`](crate::Config) can route, whether or not they are routed.
    ///
    /// Transactions, fixed by the applied configuration:
    ///
    /// - without high-g events: 1, `[Write(0x9B), Read(4)]` over FIFO_STATUS1, FIFO_STATUS2,
    ///   ALL_INT_SRC and STATUS_REG (0x1B to 0x1E);
    /// - with high-g events: 3, FIFO_STATUS1 and FIFO_STATUS2 (`Read(2)`), HG_WAKE_UP_SRC at 0x4C
    ///   (`Read(1)`), STATUS_REG at 0x1E (`Read(1)`). ALL_INT_SRC is not read: by default reading
    ///   it resets the latched interrupt signals unless FUNCTIONS_ENABLE DIS_RST_LIR_ALL_INT is 1
    ///   (9.55, p93), and a flag cleared before its own status register is read would be lost.
    ///
    /// Cleared by the read (datasheet): FIFO_OVR_LATCHED and COUNTER_BDR_IA when FIFO_STATUS2 is
    /// read (9.26, p75); the latched interrupt signals when ALL_INT_SRC is read (9.55, p93; read
    /// only in the first case, where this configuration enables none of them). The datasheet says
    /// nothing about clearing on reading STATUS_REG (9.28, p77), HG_WAKE_UP_SRC (9.51, p90) or the
    /// DIFF_FIFO count, and the FIFO_WTM_IA, FIFO_OVR_IA and FIFO_FULL_IA flags are states. An
    /// overrun shown is counted in [`Stats::fifo_overruns`].
    ///
    /// A failed or cancelled call that had already read FIFO_STATUS2 has consumed the latched
    /// overrun flag: it is counted in the stats but the call returns no report of it.
    /// [`Error::NotConfigured`] before any bus access.
    pub async fn interrupt_status(&mut self) -> Result<InterruptStatus, Error<SPI::Error>> {
        let high_g_events = match self.applied.as_ref() {
            Some(a) => a.config.high_g_events.is_some(),
            None => return Err(Error::NotConfigured),
        };
        let (fifo, hg_src, status);
        if high_g_events {
            fifo = self.read_fifo_status().await?.flags;
            hg_src = self.read_register(reg::HG_WAKE_UP_SRC).await?;
            status = self.read_register(reg::STATUS_REG).await?;
        } else {
            let mut b = [0u8; 4];
            self.read_registers(reg::FIFO_STATUS1, &mut b).await?;
            let [_status1, status2, _all_int_src, status_reg] = b;
            fifo = FifoFlags::from_status2(status2);
            self.note_flags(fifo);
            hg_src = 0;
            status = status_reg;
        }
        Ok(InterruptStatus {
            accel_data_ready: status & reg::STATUS_XLDA != 0,
            gyro_data_ready: status & reg::STATUS_GDA != 0,
            high_g_data_ready: status & reg::STATUS_XLHGDA != 0,
            timestamp_overflow: status & reg::STATUS_TIMESTAMP_ENDCOUNT != 0,
            fifo,
            high_g_wake_up: hg_src & reg::HG_WAKE_UP_SRC_HG_WU_IA != 0,
            high_g_shock: hg_src & reg::HG_WAKE_UP_SRC_HG_SHOCK_CHANGE_IA != 0,
        })
    }
}
