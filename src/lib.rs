#![no_std]

#[macro_use]
extern crate alloc;

use modular_bitfield::prelude::*;
use usb_device::class_prelude::*;

mod control;
mod descriptors;

use crate::control::*;
use crate::descriptors::*;

#[derive(PartialEq, Debug)]
enum ControlType {
    Current,
    Range,
    Memory,
}

impl ControlType {
    fn from(raw: u8) -> ControlType {
        match raw {
            0x1 => ControlType::Current,
            0x2 => ControlType::Range,
            0x3 => ControlType::Memory,
            _ => {
                panic!("Unrecognized raw type");
            }
        }
    }
}

trait ControlReplyWriter {
    fn layout1_cur(self, value: u8);
    fn layout2_cur(self, value: u16);
    fn layout3_cur(self, value: u32);

    fn layout1_range(self, ranges: &[ControlRange<u8>]);
    fn layout2_range(self, ranges: &[ControlRange<u16>]);
    fn layout3_range(self, ranges: &[ControlRange<u32>]);
}

impl<B: usb_device::bus::UsbBus> ControlReplyWriter for ControlIn<'_, '_, '_, B> {
    fn layout1_cur(self, value: u8) {
        self.accept_with(&[value])
            .expect("Couldn't accept transfer!");
    }

    fn layout2_cur(self, value: u16) {
        self.accept_with(&value.to_le_bytes())
            .expect("Couldn't accept transfer");
    }

    fn layout3_cur(self, value: u32) {
        self.accept_with(&value.to_le_bytes())
            .expect("Couldn't accept transfer");
    }

    fn layout1_range(self, ranges: &[ControlRange<u8>]) {
        accept_get_with(self, ranges).expect("Couldn't accept transfer");
    }

    fn layout2_range(self, ranges: &[ControlRange<u16>]) {
        accept_get_with(self, ranges).expect("Couldn't accept transfer");
    }

    fn layout3_range(self, ranges: &[ControlRange<u32>]) {
        accept_get_with(self, ranges).expect("Couldn't accept transfer");
    }
}

trait ControlEntity {
    fn entity_id(&self) -> u8;

    fn get_cur<W: ControlReplyWriter>(&self, control_selector: u8, reply_writer: W);
    fn get_range<W: ControlReplyWriter>(&self, control_selector: u8, reply_writer: W);
}

pub struct ClockSource {
    entity_id: u8,
}

impl ClockSource {
    pub fn new(entity_id: u8) -> ClockSource {
        ClockSource { entity_id }
    }

    fn write_descriptor(&self, writer: &mut usb_device::descriptor::DescriptorWriter) {
        let CS_INTERFACE = 0x24;
        writer
            .write(
                CS_INTERFACE,
                &descriptors::ClockSource::new()
                    .with_subtype(Subtype::ClockSource)
                    .with_clock_id(self.entity_id)
                    .with_clock_type(ClockType::InternalFixed)
                    .with_sync_to_sof(false)
                    .with_associated_terminal_id(0)
                    .with_name(0)
                    .with_frequency(Control::ReadWrite)
                    .into_bytes(),
            )
            .unwrap();
    }
}

impl ControlEntity for ClockSource {
    fn entity_id(&self) -> u8 {
        self.entity_id
    }

    fn get_cur<W: ControlReplyWriter>(&self, control_selector: u8, reply_writer: W) {
        if control_selector == 1 {
            // CS_SAM_FREQ_CONTROL
            reply_writer.layout3_cur(44_100);
        } else {
            // TODO: Handle unrecognized controls
        }
    }

    fn get_range<W: ControlReplyWriter>(&self, control_selector: u8, reply_writer: W) {
        if control_selector == 1 {
            reply_writer.layout3_range(&[ControlRange::<u32> {
                min: 44100,
                max: 44100,
                res: 0,
            }]);
        } else {
            // TODO: Handle unrecognized controls
        }
    }
}

pub struct UsbAudio<'a, 'b, B: usb_device::bus::UsbBus> {
    pub nbytes: usize,
    pub nbytes_sent: usize,
    pub iface: usb_device::class_prelude::InterfaceNumber,
    pub alt_setting_2: u8,
    pub iface2: InterfaceNumber,
    pub alt_setting_3: u8,
    pub iface3: InterfaceNumber,
    pub data_ep: usb_device::endpoint::EndpointOut<'a, B>,
    pub source_ep: usb_device::endpoint::EndpointIn<'a, B>,
    pub clocks: &'b [ClockSource],
    pub samps: alloc::vec::Vec<u16>,
    pub usb_overruns: usize,
}

impl<'a, 'b, B: usb_device::bus::UsbBus> UsbAudio<'a, 'b, B> {
    pub fn new(
        alloc: &'a usb_device::bus::UsbBusAllocator<B>,
        max_packet_size: u16,
        clocks: &'b [ClockSource],
    ) -> UsbAudio<'a, 'b, B> {
        /*let mut data = [0; 100];
        for i in 0..100 {
            let x = (i as f32).sin() * 127.;
            let x = x as i8;
            data[i] = x as u8;
        }*/
        UsbAudio {
            nbytes: 0,
            nbytes_sent: 0,
            iface: alloc.interface(),
            alt_setting_2: 0,
            iface2: alloc.interface(),
            alt_setting_3: 0,
            iface3: alloc.interface(),
            // TODO: usb_device should let us explicitly allocate a data EP & feedback
            // EP with appropriate EP numbers
            data_ep: alloc.isochronous(
                IsochronousSynchronizationType::Adaptive,
                IsochronousUsageType::Data,
                200,
                1,
            ),
            source_ep: alloc.isochronous(
                IsochronousSynchronizationType::Asynchronous,
                IsochronousUsageType::Data,
                45,
                1,
            ),
            clocks: clocks,
            samps: vec![],
            usb_overruns: 0,
            //mic_data: data,
        }
    }
}

impl<'a, 'b, B: usb_device::bus::UsbBus> usb_device::class::UsbClass<B> for UsbAudio<'a, 'b, B> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()> {
        /*
        The design of this audio device is as follows:

        USB IN  <--> Speakers
        USB OUT <--> Feature Unit <--> Microphone
        */

        writer.iad(
            self.iface, 3, 1, /* AUDIO */
            0, /* CONTROL */
            0x20,
        )?;
        writer.interface(self.iface, 1 /* AUDIO */, 1 /* CONTROL */, 0x20)?;
        let CS_INTERFACE = 0x24;
        // Setup the audio control
        // Clock sources are 8 bytes. Input terminals are 17. Output terminals are 12.
        writer.write(
            CS_INTERFACE,
            &AudioClass::new()
                .with_subtype(Subtype::Header)
                .with_spec_number_major(0)
                .with_spec_number_minor(2)
                .with_category(0xff)
                .with_total_length(8 + 17 * 2 + 12 * 2)
                .with_latency_control(0)
                .into_bytes(),
        )?;

        // USB IN <--> Speakers route
        // USB IN
        writer.write(
            CS_INTERFACE,
            &InputTerminal::new()
                .with_subtype(Subtype::InputTerminal)
                .with_terminal_id(1)
                .with_terminal_type(0x0101)
                .with_associated_terminal_id(0)
                .with_clock_source_id(2)
                .with_num_channels(2)
                .with_channel_config(ChannelConfig::new().with_channels(3))
                .with_channel_names(0)
                .with_controls(InputTerminalControls::new())
                .with_terminal(0)
                .into_bytes(),
        )?;
        for clock in self.clocks.iter() {
            clock.write_descriptor(writer);
        }

        // Speakers
        writer.write(
            CS_INTERFACE,
            &OutputTerminal::new()
                .with_subtype(Subtype::OutputTerminal)
                .with_terminal_id(3)
                .with_terminal_type(0x0301)
                .with_associated_terminal_id(0)
                .with_source_id(1)
                .with_clock_source_id(2)
                .with_name(0)
                .into_bytes(),
        )?;

        // USB OUT <--> Feature Unit <--> Microphone route
        // USB OUT
        writer.write(
            CS_INTERFACE,
            &OutputTerminal::new()
                .with_subtype(Subtype::OutputTerminal)
                .with_terminal_id(4)
                .with_terminal_type(0x0101)
                .with_associated_terminal_id(0)
                .with_source_id(5)
                .with_clock_source_id(2)
                .with_name(0)
                .into_bytes(),
        )?;

        // Microphone
        writer.write(
            CS_INTERFACE,
            &InputTerminal::new()
                .with_subtype(Subtype::InputTerminal)
                .with_terminal_id(5)
                .with_terminal_type(0x0201)
                .with_associated_terminal_id(0)
                .with_clock_source_id(2)
                .with_num_channels(1)
                .with_channel_config(ChannelConfig::new().with_channels(1))
                .with_channel_names(0)
                .with_controls(InputTerminalControls::new())
                .with_terminal(0)
                .into_bytes(),
        )?;

        // Feature unit in Microphone path
        /*writer.write(
            CS_INTERFACE,
            &[
                6, // FEATURE_UNIT
                6, // Unit ID
                5, // Source ID
                0x0, 0x0, 0x0, 0x0, // bmaControls
                0,   // iFeature
            ],
        )?;*/

        // Setup the speaker streaming
        //writer.iad(self.iface2, 1, 1 /* AUDIO */, 2 /* STREAMING */, 0)?;
        writer.interface_alt(self.iface2, 0, 1, 2, 0x20, None)?;
        writer.interface_alt(
            self.iface2,
            1, /* alt */
            1, /* AUDIO */
            2, /* STREAMING */
            0x20,
            None,
        )?;
        writer.write(
            CS_INTERFACE,
            &AudioStreamingInterface::new()
                .with_subtype(StreamingSubtype::General)
                .with_terminal_id(1)
                .with_format_type(FormatType::Type1)
                .with_formats(Formats::new().with_pcm(true))
                .with_num_channels(2)
                .with_channel_config(ChannelConfig::new().with_channels(3))
                .with_channel_names(0)
                .into_bytes(),
        )?;
        writer.write(
            CS_INTERFACE,
            &Type1Format::new()
                .with_subtype(StreamingSubtype::FormatType)
                .with_format_type(FormatType::Type1)
                .with_subslot_size(2)
                .with_bit_resolution(16)
                .into_bytes(),
        )?;

        writer.endpoint(&self.data_ep)?;

        // Class specific endpoint descriptor
        writer.write(
            0x25, // CS_ENDPOINT
            &AudioStreamingEndpoint::new()
                .with_subtype(EndpointDescriptorSubtype::General)
                .into_bytes(),
        )?;

        // Setup the microphone streaming
        writer.interface_alt(self.iface3, 0, 1, 2, 0x20, None)?;
        writer.interface_alt(
            self.iface3,
            1, /* alt */
            1, /* AUDIO */
            2, /* STREAMING */
            0x20,
            None,
        )?;
        writer.write(
            CS_INTERFACE,
            &AudioStreamingInterface::new()
                .with_subtype(StreamingSubtype::General)
                .with_terminal_id(4)
                .with_format_type(FormatType::Type1)
                .with_formats(Formats::new().with_pcm(true))
                .with_num_channels(1)
                .with_channel_config(ChannelConfig::new().with_channels(1))
                .with_channel_names(0)
                .into_bytes(),
        )?;
        writer.write(
            CS_INTERFACE,
            &Type1Format::new()
                .with_subtype(StreamingSubtype::FormatType)
                .with_format_type(FormatType::Type1)
                .with_subslot_size(1)
                .with_bit_resolution(8)
                .into_bytes(),
        )?;

        writer.endpoint(&self.source_ep)?;

        // Class specific endpoint descriptor
        writer.write(
            0x25, // CS_ENDPOINT
            &AudioStreamingEndpoint::new()
                .with_subtype(EndpointDescriptorSubtype::General)
                .into_bytes(),
        )?;

        Ok(())
    }

    fn get_alt_setting(&mut self, interface: InterfaceNumber) -> Option<u8> {
        let i: u8 = interface.into();
        log::info!("get_alt_setting(iface=0x{:x})", i);
        if interface == self.iface2 {
            Some(self.alt_setting_2)
        } else if interface == self.iface3 {
            Some(self.alt_setting_3)
        } else {
            None
        }
    }

    fn set_alt_setting(&mut self, interface: InterfaceNumber, alt: u8) -> bool {
        let i: u8 = interface.into();
        log::info!("set_alt_setting(iface=0x{:x}, alt={})", i, alt);
        if interface == self.iface2 {
            self.alt_setting_2 = alt;
            true
        } else if interface == self.iface3 {
            self.alt_setting_3 = alt;
            true
        } else {
            false
        }
    }

    fn control_in(&mut self, xfer: ControlIn<'_, '_, '_, B>) {
        let req = xfer.request();
        if req.request_type == usb_device::control::RequestType::Class
            && req.recipient == usb_device::control::Recipient::Interface
        {
            let reqtype = ControlType::from(req.request);
            let control_selector = (req.value >> 8) as u8;
            let channel_number = req.value & 0xFF;
            let interface = req.index & 0xFF;
            let entity_id = req.index >> 8;

            for control in self.clocks.iter() {
                if control.entity_id() == entity_id as u8 {
                    match reqtype {
                        ControlType::Range => {
                            control.get_range(control_selector, xfer);
                        }
                        ControlType::Current => {
                            control.get_cur(control_selector, xfer);
                        }
                        _ => {
                            panic!("Unrecognized ControlType request!");
                        }
                    }
                    return;
                }
            }

            log::info!(
                "unhandled audio GET: {:?} CS={} chan={} iface={} eid={}",
                reqtype,
                control_selector,
                channel_number,
                interface,
                entity_id
            );
        } else {
            log::info!(
                "control: dir={:?} reqtype={:?} recip={:?} req={} value={} idx={} len={}",
                req.direction,
                req.request_type,
                req.recipient,
                req.request,
                req.value,
                req.index,
                req.length
            );
        }
    }

    fn control_out(&mut self, xfer: ControlOut<'_, '_, '_, B>) {
        /*
        bytes recv'd = 1174468
        [INFO audio]: Num bytes recv'd = 1218568
        [INFO audio]: control: dir=Out reqtype=Standard recip=Interface req=11 value=0 idx=2 len=0
        */

        let req = xfer.request();
        log::info!(
            "control: dir={:?} reqtype={:?} recip={:?} req={} value={} idx={} len={}",
            req.direction,
            req.request_type,
            req.recipient,
            req.request,
            req.value,
            req.index,
            req.length
        );
        /*if req.request_type == usb_device::control::RequestType::Class && req.recipient == usb_device::control::Recipient::Interface {
        let reqtype = ControlType::from(req.request);
        let control_selector = req.value >> 8;
        let channel_number = req.value & 0xFF;
        let interface = req.index & 0xFF;
        let entity_id = req.index >> 8;

        log::info!("audio SET: {:?} CS={} chan={} iface={} eid={}", reqtype, control_selector, channel_number, interface, entity_id);*/
        //log::info!("control: dir={:?} reqtype={:?} recip={:?} req={} value={} idx={} len={}", req.direction, req.request_type, req.recipient, req.request, req.value, req.index, req.length);
        if req.recipient == usb_device::control::Recipient::Interface
            && req.request == 11
            && req.index == 1
        {
            //log::info!("Handling");
            //xfer.reject().expect("Couldn't reject");
            xfer.accept().expect("Couldn't accept");
        } else if req.recipient == usb_device::control::Recipient::Interface
            && req.request == 11
            && req.index == 2
        {
            //log::info!("Handling");
            //xfer.reject().expect("Couldn't reject");
            xfer.accept().expect("Couldn't accept");
        } else if req.recipient == usb_device::control::Recipient::Interface
            && req.request_type == usb_device::control::RequestType::Class
            && req.request == 1
            && req.value == 256
            && req.index == 512
        {
            // Setting the Clock Valid control?
            //log::info!("{:?}", req.);
            xfer.accept().expect("Couldn't accept");
        }
        /* else if req.recipient == usb_device::control::Recipient::Endpoint && req.request == 1 && req.index == 1 {
            xfer.accept().expect("Couldn't reject");
        } else if req.recipient == usb_device::control::Recipient::Endpoint && req.request == 1 && req.index == 129 {
            xfer.accept().expect("Couldn't reject");
        }*/
        //}
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        if self.data_ep.address().index() == addr.index() {
            let mut buf = [0; 768];
            let nbytes = match self.data_ep.read(&mut buf) {
                Ok(x) => x,
                Err(e) => {
                    log::error!("Received error {:?}", e);
                    return;
                }
            };
            //log::info!("RX'd data: EP{}", addr.index());
            //log::info!("{}", buf[5]);
            self.nbytes += nbytes;

            if self.samps.len() < 512 {
                //self.samps.append(&buf[..]);
                for i in 0..nbytes / 2 {
                    let hi = buf[i * 2 + 1];
                    let lo = buf[i * 2];
                    self.samps.push(((hi as u16) << 8) | lo as u16);
                }
            } else {
                self.usb_overruns += nbytes;
            }
        }
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        //if addr.index() == 1 {
        //self.cnt += 1;
        //self.data_ep.write(&[5; 64]);
        self.nbytes_sent += 45; //self.mic_data.len();
                                //self.source_ep.write(&self.mic_data);
                                //}
    }
}
