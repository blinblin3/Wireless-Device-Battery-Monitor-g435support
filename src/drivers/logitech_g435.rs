//! Logitech G435 LIGHTSPEED headset (VID 046D, PID 0ACB / 0ADF) over its 2.4 GHz dongle.
//!
//! Protocol from deddroid/logitech-g435-battery (MIT) and endes "USB HID down the rabbit hole".
//! Uses vendor collection FFC0, report ID 6 (64-byte feature reports) via the Realtek bridge.
//! Entering bootloader mode briefly cuts audio (~1 s); EXIT_BOOTLOADER is always sent via Drop.

use std::collections::HashMap;
use std::sync::Mutex;
use std::thread;
use std::time::{Duration, Instant};

use crate::device::{DeviceKind, DeviceStatus, PollContext};
use crate::hid::{HidColl, HidDev};
use crate::util::{Deadline, hex4, interpolate};

const VID: u16 = 0x046D;
const PIDS: &[u16] = &[0x0ACB, 0x0ADF];
const PAGE: u16 = 0xFFC0;

const REPORT_ID: u8 = 0x06;
const ENTER_BOOTLOADER: [u8; 2] = [0x01, 0x03];
const EXIT_BOOTLOADER: [u8; 2] = [0x08, 0x03];
const SEND_COMMAND: [u8; 2] = [0x04, 0x03];

const ACK_OK: u8 = 0x5A;
const ACK_ALREADY: u8 = 0xFE;

const CAT_OTA: u8 = 0x06;
const CAT_WIRELESS: u8 = 0x0E;

const GET_MODE: u8 = 0xE0;
const SET_MODE: u8 = 0xE1;
const GET_CONNECT: u8 = 0xE3;

/// Set to `true` to automatically switch remote mode to 1 if needed (equivalent to --allow-set-mode).
const ALLOW_SET_MODE: bool = true;

/// Minimum interval between bootloader probes when the headset is offline (avoids audio cuts every 10 s).
const OFFLINE_COOLDOWN: Duration = Duration::from_secs(45);

const CURVE_MV: [i32; 22] = [
    4200, 4150, 4100, 4050, 4000, 3950, 3900, 3850, 3800, 3750, 3720, 3705, 3700, 3690, 3675, 3650, 3600, 3550, 3500, 3450, 3400, 3300,
];
const CURVE_PCT: [i32; 22] = [
    100, 95, 90, 85, 80, 75, 70, 62, 55, 41, 34, 29, 27, 17, 11, 9, 7, 5, 4, 2, 1, 0,
];

pub fn percent(mv: i32) -> i32 {
    interpolate(mv, &CURVE_MV, &CURVE_PCT)
}

pub fn parse_remote(buf: &[u8], cmd: u8) -> Option<u8> {
    buf.windows(4)
        .find(|w| w[0] == 0x02 && w[1] == 0x00 && w[2] == cmd)
        .map(|w| w[3])
}

pub fn parse_battery(buf: &[u8]) -> Option<i32> {
    for w in buf.windows(7) {
        if w[0] == 0x07 && w[1] == 0x00 && w[2] == CAT_OTA && (w[3] & 0x80) != 0 && w[4] == 0x00 {
            let mv = u16::from_le_bytes([w[5], w[6]]) as i32;
            if (2500..=4600).contains(&mv) {
                return Some(mv);
            }
        }
    }
    None
}

fn bridge(d: &HidDev, op: &[u8], body: &[u8]) -> Option<u8> {
    let mut report = Vec::with_capacity(3 + body.len());
    report.push(REPORT_ID);
    report.extend_from_slice(op);
    report.extend_from_slice(body);

    if !d.set_feature(&report) {
        return None;
    }

    let end = Deadline::in_ms(800);
    while !end.passed() {
        if let Some(data) = d.read(end.left_ms().min(100)) {
            if data.len() >= 2 && data[0] == REPORT_ID {
                return Some(data[1]);
            }
        }
    }
    None
}

fn drain(d: &HidDev) {
    for _ in 0..16 {
        match d.get_feature(REPORT_ID) {
            Some(buf) if buf.get(1) == Some(&0xFF) => break,
            None => break,
            _ => {}
        }
    }
}

fn query<F, T>(d: &HidDev, seq: u8, category: u8, length: u8, payload: &[u8], parse: F) -> Option<T>
where
    F: Fn(&[u8]) -> Option<T>,
{
    let mut cmd = Vec::with_capacity(5 + payload.len());
    cmd.extend_from_slice(&[0x50, 0x41, category, seq, length]);
    cmd.extend_from_slice(payload);

    let mut body = Vec::with_capacity(1 + cmd.len());
    body.push(cmd.len() as u8);
    body.extend_from_slice(&cmd);

    bridge(d, &SEND_COMMAND, &body)?;

    let end = Deadline::in_ms(1500);
    while !end.passed() {
        if let Some(buf) = d.get_feature(REPORT_ID) {
            if buf.get(1).copied().unwrap_or(0xFF) != 0xFF {
                if let Some(val) = parse(&buf) {
                    return Some(val);
                }
                continue;
            }
        }
        thread::sleep(Duration::from_millis(30));
    }
    None
}

/// Ensures EXIT_BOOTLOADER is always sent when leaving the scope.
struct BootloaderGuard<'a>(&'a HidDev);

impl Drop for BootloaderGuard<'_> {
    fn drop(&mut self) {
        let _ = bridge(self.0, &EXIT_BOOTLOADER, &[]);
    }
}

#[derive(Debug, PartialEq, Eq)]
enum ReadResult {
    Battery(i32),
    Offline,
    Failed,
}

fn read_dongle(c: &HidColl) -> ReadResult {
    let Some(d) = HidDev::open(c) else {
        crate::util::debug_log(|| "g435: failed to open HID device".into());
        return ReadResult::Failed;
    };
    d.flush();

    let Some(ack) = bridge(&d, &ENTER_BOOTLOADER, &[]) else {
        crate::util::debug_log(|| "g435: no ACK on ENTER_BOOTLOADER".into());
        return ReadResult::Failed;
    };
    crate::util::debug_log(|| format!("g435: enter bootloader ACK = {ack:02X}"));
    if ack != ACK_OK && ack != ACK_ALREADY {
        return ReadResult::Failed;
    }
    let _guard = BootloaderGuard(&d);

    drain(&d);

    let Some(connected) = query(&d, 1, CAT_WIRELESS, 1, &[GET_CONNECT], |b| parse_remote(b, GET_CONNECT)) else {
        crate::util::debug_log(|| "g435: no reply on GET_CONNECT".into());
        return ReadResult::Failed;
    };
    crate::util::debug_log(|| format!("g435: connected = {connected}"));
    if connected == 0 {
        return ReadResult::Offline;
    }

    let Some(mode) = query(&d, 2, CAT_WIRELESS, 1, &[GET_MODE], |b| parse_remote(b, GET_MODE)) else {
        crate::util::debug_log(|| "g435: no reply on GET_MODE".into());
        return ReadResult::Failed;
    };
    crate::util::debug_log(|| format!("g435: mode = {mode}"));

    let mut seq = 3;
    if mode != 1 {
        if !ALLOW_SET_MODE {
            crate::util::debug_log(|| "g435: mode != 1 and ALLOW_SET_MODE is false".into());
            return ReadResult::Failed;
        }
        let _ = query(&d, seq, CAT_WIRELESS, 2, &[SET_MODE, 0x01], |_| Some(()));
        seq += 1;
    }

    match query(&d, seq, CAT_OTA, 1, &[0x00, 0x31], parse_battery) {
        Some(mv) => {
            crate::util::debug_log(|| format!("g435: battery = {mv} mV"));
            ReadResult::Battery(mv)
        }
        None => {
            crate::util::debug_log(|| "g435: no reply on GET_BATTERY".into());
            ReadResult::Failed
        }
    }
}


static LAST_OFFLINE_CHECK: Mutex<Option<HashMap<String, Instant>>> = Mutex::new(None);

fn status(pid: u16) -> DeviceStatus {
    DeviceStatus {
        key: format!("logi-g435:{}", hex4(pid)),
        name: "Logitech G435".into(),
        kind: DeviceKind::Headset,
        via: "2.4 GHz dongle".into(),
        grace_sec: 60,
        vid: VID,
        hw: Some((VID, pid)),
        ..Default::default()
    }
}

pub fn poll(ctx: &mut PollContext) {
    for (vid, pid) in ctx.products(VID) {
        if !PIDS.contains(&pid) || ctx.is_claimed(vid, pid) {
            continue;
        }
        let colls = ctx.find(vid, pid, |c| c.page() == PAGE && c.feat_len() >= 64);
        let Some(c) = colls.first() else {
            continue;
        };

        // Throttle bootloader entry while the headset is known to be off.
        {
            let guard = LAST_OFFLINE_CHECK.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(last) = guard.as_ref().and_then(|m| m.get(&c.path)) {
                if last.elapsed() < OFFLINE_COOLDOWN {
                    ctx.add(DeviceStatus { connected: false, ..status(pid) }, vid, pid);
                    continue;
                }
            }
        }

        let res = read_dongle(c);
        let mut guard = LAST_OFFLINE_CHECK.lock().unwrap_or_else(|e| e.into_inner());
        let map = guard.get_or_insert_with(HashMap::new);

        let s = match res {
            ReadResult::Battery(mv) => {
                map.remove(&c.path);
                DeviceStatus {
                    percent: percent(mv),
                    millivolts: mv,
                    ..status(pid)
                }
            }
            ReadResult::Offline => {
                map.insert(c.path.clone(), Instant::now());
                DeviceStatus { connected: false, ..status(pid) }
            }
            ReadResult::Failed => {
                map.insert(c.path.clone(), Instant::now());
                continue;
            }
        };
        ctx.add(s, vid, pid);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn battery_curve() {
        assert_eq!(percent(4250), 100);
        assert_eq!(percent(4200), 100);
        assert_eq!(percent(3800), 55);
        assert_eq!(percent(3700), 27);
        assert_eq!(percent(3300), 0);
        assert_eq!(percent(3100), 0);
    }

    #[test]
    fn parse_replies() {
        let mut remote_buf = [0xFFu8; 64];
        remote_buf[2..6].copy_from_slice(&[0x02, 0x00, GET_CONNECT, 0x01]);
        assert_eq!(parse_remote(&remote_buf, GET_CONNECT), Some(1));
        assert_eq!(parse_remote(&remote_buf, GET_MODE), None);

        // 3850 mV = 0x0F0A -> little endian [0x0A, 0x0F]
        let mut bat_buf = [0xFFu8; 64];
        bat_buf[4..11].copy_from_slice(&[0x07, 0x00, CAT_OTA, 0x83, 0x00, 0x0A, 0x0F]);
        assert_eq!(parse_battery(&bat_buf), Some(3850));

        // Out of range voltage
        bat_buf[9..11].copy_from_slice(&[0xD0, 0x07]); // 2000 mV
        assert_eq!(parse_battery(&bat_buf), None);
    }
}
