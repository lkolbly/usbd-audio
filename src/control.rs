#![allow(dead_code)]

use core::convert::TryInto;
use usb_device::class_prelude::*;

#[derive(PartialEq, Debug)]
enum RequestType {
    Current,
    Range,
    Memory,
}

impl RequestType {
    fn from(raw: u8) -> RequestType {
        match raw {
            0x1 => RequestType::Current,
            0x2 => RequestType::Range,
            0x3 => RequestType::Memory,
            _ => {
                panic!("Unrecognized raw type");
            }
        }
    }
}

pub struct ControlGetRequest {
    reqtype: RequestType,
    entity_id: u8,
    control: u8,
    channel: u8,
}

pub fn parse_control_in(req: &usb_device::control::Request) -> Option<(u8, ControlGetRequest)> {
    if req.request_type == usb_device::control::RequestType::Class
        && req.recipient == usb_device::control::Recipient::Interface
    {
        let reqtype = RequestType::from(req.request);
        let control_selector = req.value >> 8;
        let channel_number = req.value & 0xFF;
        let interface = req.index & 0xFF;
        let entity_id = req.index >> 8;
        Some((
            interface.try_into().unwrap(),
            ControlGetRequest {
                reqtype,
                entity_id: entity_id.try_into().unwrap(),
                control: control_selector.try_into().unwrap(),
                channel: channel_number.try_into().unwrap(),
            },
        ))
    } else if req.request_type == usb_device::control::RequestType::Class
        && req.recipient == usb_device::control::Recipient::Endpoint
    {
        // TODO: Implement this
        None
    } else {
        None
    }
}

/// This type is used with 8-bit, 16-bit, or 32-bit types, corresponding to the
/// "Layout 1", "Layout 2", and "Layout 3" control layouts, respectively.
//#[repr(packed(1))]
pub struct ControlRange<T> {
    pub min: T,
    pub max: T,
    pub res: T,
}

impl<T> ControlRange<T> {
    fn size() -> usize {
        core::mem::size_of::<T>() * 3
    }
}

fn pack_rangelist_into_buf<T>(ranges: &[ControlRange<T>], buf: &mut [u8]) -> usize {
    // TODO: Make this a graceful error
    assert!(buf.len() >= 2 + ranges.len() * ControlRange::<T>::size());

    let nranges = ranges.len();
    buf[0] = (nranges & 0xFF).try_into().unwrap();
    buf[1] = ((nranges >> 8) & 0xFF).try_into().unwrap();

    let src: *const ControlRange<T> = ranges.as_ptr();
    let dst: *mut ControlRange<T> = unsafe { buf.as_mut_ptr().offset(2) as *mut ControlRange<T> };
    unsafe {
        core::ptr::copy_nonoverlapping(src, dst, nranges);
    }
    2 + ranges.len() * ControlRange::<T>::size()
}

pub fn accept_get_with<T, B: usb_device::bus::UsbBus>(
    xfer: ControlIn<'_, '_, '_, B>,
    ranges: &[ControlRange<T>],
) -> usb_device::Result<()> {
    xfer.accept(|buf| Ok(pack_rangelist_into_buf(ranges, buf)))?;
    Ok(())
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn pack_works() {
        let mut buf = [0; 14];
        let nbytes = pack_rangelist_into_buf(
            &[ControlRange::<u32> {
                min: 44100,
                max: 44100,
                res: 0xaabbccdd,
            }],
            &mut buf,
        );
        assert_eq!(nbytes, 14);
        assert_eq!(
            buf,
            [
                0x1, 0x0, // 1 subrange
                0x44, 0xac, 0x00, 0x00, // MIN
                0x44, 0xac, 0x00, 0x00, // MAX
                0xdd, 0xcc, 0xbb, 0xaa, // RES
            ]
        );
    }
}
