#![no_std]

#[macro_use]
extern crate alloc;

use modular_bitfield::prelude::*;
use usb_device::class_prelude::*;

mod channels;
mod control;
mod descriptors;

pub use crate::channels::*;
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

pub trait ControlEntity<B: usb_device::bus::UsbBus> {
    fn entity_id(&self) -> u8;

    fn write_descriptor(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()>;

    fn get_cur(&self, control_selector: u8, reply_writer: ControlIn<'_, '_, '_, B>);
    fn get_range(&self, control_selector: u8, reply_writer: ControlIn<'_, '_, '_, B>);
}

pub trait StreamableEntity {
    fn write_stream_iface(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()>;

    fn ep_out(&mut self, addr: EndpointAddress) -> usb_device::Result<bool>;
    fn ep_in_complete(&mut self, addr: EndpointAddress) -> usb_device::Result<bool>;
}

/// Trait for entities which output audio
pub trait AudioSourceEntity {
    fn entity_id(&self) -> u8;
}

pub struct EntityAllocator {
    primary_iface: InterfaceNumber,
    next_entity_id: core::cell::RefCell<u8>,
}

impl EntityAllocator {
    pub fn new<B: usb_device::bus::UsbBus>(alloc: &usb_device::bus::UsbBusAllocator<B>) -> Self {
        Self {
            primary_iface: alloc.interface(),
            next_entity_id: core::cell::RefCell::new(1),
        }
    }

    fn next_eid(&self) -> u8 {
        let eid = *self.next_entity_id.borrow();
        *self.next_entity_id.borrow_mut() += 1;
        eid
    }

    pub fn make_clock(&self) -> ClockSource {
        ClockSource {
            entity_id: self.next_eid(),
            _lifetime: core::marker::PhantomData,
        }
    }

    pub fn make_usb_stream_source<'audio, 'usb, B: usb_device::bus::UsbBus>(
        &'audio self,
        alloc: &'usb usb_device::bus::UsbBusAllocator<B>,
        clock: &ClockSource<'audio>,
        channels: ChannelSet,
    ) -> UsbStreamSource<'audio, 'usb, B> {
        let eid = self.next_eid();
        let nchannels = channels.count() as usize;
        UsbStreamSource {
            entity_id: eid,
            clock_id: clock.entity_id,
            channels: channels,
            endpoint: alloc.isochronous(
                IsochronousSynchronizationType::Adaptive,
                IsochronousUsageType::Data,
                200,
                1,
            ),
            iface: alloc.interface(),
            samps: SampleBuffer::new(nchannels),
            nbytes_read: 0,
            _allocator: core::marker::PhantomData,
        }
    }

    pub fn make_usb_stream_sink<'audio, 'usb, B: usb_device::bus::UsbBus>(
        &'audio self,
        alloc: &'usb usb_device::bus::UsbBusAllocator<B>,
        source: &AudioSourceEntity,
        clock: &ClockSource<'audio>,
    ) -> UsbStreamSink<'audio, 'usb, B> {
        UsbStreamSink {
            entity_id: self.next_eid(),
            source_id: source.entity_id(),
            clock_id: clock.entity_id,
            endpoint: alloc.isochronous(
                IsochronousSynchronizationType::Asynchronous,
                IsochronousUsageType::Data,
                45,
                1,
            ),
            iface: alloc.interface(),
            samps: vec![],
            ready_to_send: true,
            _allocator: core::marker::PhantomData,
        }
    }

    pub fn make_external_audio_sink<'audio>(
        &'audio self,
        clock: &ClockSource<'audio>,
        source: &AudioSourceEntity,
    ) -> ExternalAudioSink<'audio> {
        ExternalAudioSink {
            entity_id: self.next_eid(),
            clock_id: clock.entity_id,
            source_id: source.entity_id(),
            _allocator: core::marker::PhantomData,
        }
    }

    pub fn make_external_audio_source(
        &self,
        clock: &ClockSource,
        channels: ChannelSet,
    ) -> ExternalAudioSource {
        ExternalAudioSource {
            entity_id: self.next_eid(),
            clock_id: clock.entity_id,
            channels: channels,
            _allocator: core::marker::PhantomData,
        }
    }
}

pub struct ClockSource<'a> {
    pub entity_id: u8,
    _lifetime: core::marker::PhantomData<&'a u8>,
}

impl<'a, B: usb_device::bus::UsbBus> ControlEntity<B> for ClockSource<'a> {
    fn entity_id(&self) -> u8 {
        self.entity_id
    }

    fn write_descriptor(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()> {
        let CS_INTERFACE = 0x24;
        writer.write(
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
    }

    fn get_cur(&self, control_selector: u8, reply_writer: ControlIn<'_, '_, '_, B>) {
        if control_selector == 1 {
            // CS_SAM_FREQ_CONTROL
            reply_writer.layout3_cur(44_100);
        } else {
            // TODO: Handle unrecognized controls
        }
    }

    fn get_range(&self, control_selector: u8, reply_writer: ControlIn<'_, '_, '_, B>) {
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

pub struct UsbStreamSource<'audio, 'usb, B: usb_device::bus::UsbBus> {
    entity_id: u8,
    clock_id: u8,
    channels: ChannelSet,
    endpoint: usb_device::endpoint::EndpointOut<'usb, B>,
    iface: InterfaceNumber,
    pub samps: SampleBuffer<u16>,
    nbytes_read: usize,
    _allocator: core::marker::PhantomData<&'audio EntityAllocator>,
}

impl<'audio, 'usb, B: usb_device::bus::UsbBus> AudioSourceEntity
    for UsbStreamSource<'audio, 'usb, B>
{
    fn entity_id(&self) -> u8 {
        self.entity_id
    }
}

impl<'audio, 'usb, B: usb_device::bus::UsbBus> UsbStreamSource<'audio, 'usb, B> {
    fn write_descriptor(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()> {
        let CS_INTERFACE = 0x24;
        writer.write(
            CS_INTERFACE,
            &InputTerminal::new()
                .with_subtype(Subtype::InputTerminal)
                .with_terminal_id(self.entity_id)
                .with_terminal_type(0x0101)
                .with_associated_terminal_id(0)
                .with_clock_source_id(self.clock_id)
                .with_num_channels(self.channels.count())
                .with_channel_config(self.channels.config())
                .with_channel_names(0)
                .with_controls(InputTerminalControls::new())
                .with_terminal(0)
                .into_bytes(),
        )
    }

    fn write_stream_iface(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()> {
        /*if self.entity_id == 4 {
            return Ok(());
        }*/
        let CS_INTERFACE = 0x24;
        writer.interface_alt(self.iface, 0, 1, 2, 0x20, None)?;
        writer.interface_alt(
            self.iface, 1, /* alt */
            1, /* AUDIO */
            2, /* STREAMING */
            0x20, None,
        )?;
        writer.write(
            CS_INTERFACE,
            &AudioStreamingInterface::new()
                .with_subtype(StreamingSubtype::General)
                .with_terminal_id(self.entity_id)
                .with_format_type(FormatType::Type1)
                .with_formats(Formats::new().with_pcm(true))
                .with_num_channels(self.channels.count())
                .with_channel_config(self.channels.config())
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

        writer.endpoint(&self.endpoint)?;

        // Class specific endpoint descriptor
        writer.write(
            0x25, // CS_ENDPOINT
            &AudioStreamingEndpoint::new()
                .with_subtype(EndpointDescriptorSubtype::General)
                .into_bytes(),
        )?;
        Ok(())
    }

    fn recv_data(&mut self) {
        let mut buf = [0; 768];
        let nbytes = match self.endpoint.read(&mut buf) {
            Ok(x) => x,
            Err(e) => {
                log::error!("Received error {:?}", e);
                return;
            }
        };
        self.nbytes_read += nbytes;

        let mut i = 0;
        let nchannels = self.channels.count() as usize;
        assert!(nchannels < 8);
        while i < nbytes {
            let mut samples: [u16; 8] = [0; 8];

            for channel in 0..nchannels {
                let hi = buf[i + 1];
                let lo = buf[i];
                let sample = ((hi as u16) << 8) | lo as u16;
                samples[channel] = sample;
                i += 2;
            }

            self.samps.push(&samples[0..nchannels]);
        }
    }
}

pub struct UsbStreamSink<'audio, 'usb, B: usb_device::bus::UsbBus> {
    entity_id: u8,
    clock_id: u8,
    source_id: u8,
    pub endpoint: usb_device::endpoint::EndpointIn<'usb, B>,
    iface: InterfaceNumber,
    pub samps: alloc::vec::Vec<u8>,
    ready_to_send: bool,
    _allocator: core::marker::PhantomData<&'audio EntityAllocator>,
}

impl<'audio, 'usb, B: usb_device::bus::UsbBus> UsbStreamSink<'audio, 'usb, B> {
    fn write_descriptor(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()> {
        let CS_INTERFACE = 0x24;
        writer.write(
            CS_INTERFACE,
            &OutputTerminal::new()
                .with_subtype(Subtype::OutputTerminal)
                .with_terminal_id(self.entity_id)
                .with_terminal_type(0x0101)
                .with_associated_terminal_id(0)
                .with_source_id(self.source_id)
                .with_clock_source_id(self.clock_id)
                .with_name(0)
                .into_bytes(),
        )
    }

    fn write_stream_iface(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()> {
        let CS_INTERFACE = 0x24;
        writer.interface_alt(self.iface, 0, 1, 2, 0x20, None)?;
        writer.interface_alt(
            self.iface, 1, /* alt */
            1, /* AUDIO */
            2, /* STREAMING */
            0x20, None,
        )?;
        writer.write(
            CS_INTERFACE,
            &AudioStreamingInterface::new()
                .with_subtype(StreamingSubtype::General)
                .with_terminal_id(self.entity_id)
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

        writer.endpoint(&self.endpoint)?;

        // Class specific endpoint descriptor
        writer.write(
            0x25, // CS_ENDPOINT
            &AudioStreamingEndpoint::new()
                .with_subtype(EndpointDescriptorSubtype::General)
                .into_bytes(),
        )?;
        Ok(())
    }

    fn endpoint_in_complete(&mut self) {
        self.ready_to_send = true;
    }

    fn poll(&mut self) {
        if self.samps.len() > 45 {
            let end = if self.samps.len() > 45 {
                45
            } else {
                self.samps.len()
            };
            match self.endpoint.write(&self.samps[..end]) {
                Ok(_) => {
                    self.samps = self.samps[end..].to_vec();
                }
                Err(usb_device::UsbError::WouldBlock) => {
                    // Would block, do nothing
                    //blocks += 1;
                    return;
                }
                Err(e) => {
                    panic!("Endpoint send error {:?}", e);
                }
            }
        }
        self.ready_to_send = false;
    }
}

pub struct ExternalAudioSink<'audio> {
    entity_id: u8,
    clock_id: u8,
    source_id: u8,
    _allocator: core::marker::PhantomData<&'audio EntityAllocator>,
}

impl<'audio> ExternalAudioSink<'audio> {
    fn write_descriptor(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()> {
        let CS_INTERFACE = 0x24;
        writer.write(
            CS_INTERFACE,
            &OutputTerminal::new()
                .with_subtype(Subtype::OutputTerminal)
                .with_terminal_id(self.entity_id)
                .with_terminal_type(0x0301)
                .with_associated_terminal_id(0)
                .with_source_id(self.source_id)
                .with_clock_source_id(self.clock_id)
                .with_name(0)
                .into_bytes(),
        )
    }
}

pub struct ExternalAudioSource<'audio> {
    entity_id: u8,
    clock_id: u8,
    channels: ChannelSet,
    _allocator: core::marker::PhantomData<&'audio EntityAllocator>,
}

impl<'audio> AudioSourceEntity for ExternalAudioSource<'audio> {
    fn entity_id(&self) -> u8 {
        self.entity_id
    }
}

impl<'audio> ExternalAudioSource<'audio> {
    fn write_descriptor(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()> {
        let CS_INTERFACE = 0x24;
        writer.write(
            CS_INTERFACE,
            &InputTerminal::new()
                .with_subtype(Subtype::InputTerminal)
                .with_terminal_id(self.entity_id)
                .with_terminal_type(0x0201)
                .with_associated_terminal_id(0)
                .with_clock_source_id(self.clock_id)
                .with_num_channels(self.channels.count())
                .with_channel_config(self.channels.config())
                .with_channel_names(0)
                .with_controls(InputTerminalControls::new())
                .with_terminal(0)
                .into_bytes(),
        )
    }
}

pub struct SampleBuffer<Sample: Copy> {
    channels: usize,
    buffer: alloc::vec::Vec<Sample>,
    pub pushed: usize,
    pub popped: usize,
}

impl<Sample: Copy> SampleBuffer<Sample> {
    pub fn new(channels: usize) -> Self {
        SampleBuffer {
            channels,
            buffer: vec![],
            pushed: 0,
            popped: 0,
        }
    }

    pub fn push(&mut self, sample: &[Sample]) {
        assert!(sample.len() == self.channels);
        if self.buffer.len() >= 512 {
            // Overflow
            // TODO: Report it
            return;
        }
        for i in 0..self.channels {
            self.buffer.push(sample[i]);
            self.pushed += 1;
        }
    }

    pub fn next<F: FnOnce(&[Sample])>(&mut self, cb: F) -> bool {
        if self.buffer.len() == 0 {
            return false;
        }
        // Samples should only be added or removed in multiples of channel
        assert!(self.buffer.len() >= self.channels);
        cb(&self.buffer[0..self.channels]);
        for _ in 0..self.channels {
            self.buffer.remove(0);
            self.popped += 1;
        }
        true
    }

    pub fn available(&self) -> usize {
        self.buffer.len()
    }
}

pub struct UsbAudio<'a, 'b, 'audio, B: usb_device::bus::UsbBus> {
    pub nbytes: usize,
    pub nbytes_sent: usize,
    pub iface: usb_device::class_prelude::InterfaceNumber,
    pub alt_setting_2: u8,
    pub alt_setting_3: u8,
    pub controls: &'b [&'b mut dyn ControlEntity<B>],
    pub stream_sources: &'b mut [UsbStreamSource<'audio, 'a, B>],
    pub stream_sinks: &'b mut [UsbStreamSink<'audio, 'a, B>],
    pub ext_audio_sinks: &'b [ExternalAudioSink<'audio>],
    pub ext_audio_sources: &'b [ExternalAudioSource<'audio>],
    pub usb_overruns: usize,
}

impl<'a, 'b, 'audio, B: usb_device::bus::UsbBus> UsbAudio<'a, 'b, 'audio, B> {
    pub fn new(
        audio_alloc: &'audio EntityAllocator,
        alloc: &'a usb_device::bus::UsbBusAllocator<B>,
        max_packet_size: u16,
        controls: &'b [&'b mut dyn ControlEntity<B>],
        stream_sources: &'b mut [UsbStreamSource<'audio, 'a, B>],
        stream_sinks: &'b mut [UsbStreamSink<'audio, 'a, B>],
        ext_audio_sinks: &'b [ExternalAudioSink<'audio>],
        ext_audio_sources: &'b [ExternalAudioSource<'audio>],
    ) -> Self {
        UsbAudio {
            nbytes: 0,
            nbytes_sent: 0,
            iface: audio_alloc.primary_iface,
            alt_setting_2: 0,
            alt_setting_3: 0,
            controls: controls,
            stream_sources: stream_sources,
            stream_sinks: stream_sinks,
            ext_audio_sinks: ext_audio_sinks,
            ext_audio_sources: ext_audio_sources,
            usb_overruns: 0,
        }
    }

    pub fn poll(&mut self) {
        for sink in self.stream_sinks.iter_mut() {
            sink.poll();
        }
    }
}

impl<'a, 'b, 'audio, B: usb_device::bus::UsbBus> usb_device::class::UsbClass<B>
    for UsbAudio<'a, 'b, 'audio, B>
{
    fn get_configuration_descriptors(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()> {
        /*
        The design of this audio device is as follows:

        USB IN  <--> Speakers
        USB OUT <--> Feature Unit <--> Microphone
        */

        log::info!("Writing config descriptor");

        writer.iad(
            self.iface, 4, 1, /* AUDIO */
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
                .with_total_length(
                    (8 * self.controls.len()
                        + 17 * self.stream_sources.len()
                        + 17 * self.ext_audio_sources.len()
                        + 12 * self.stream_sinks.len()
                        + 12 * self.ext_audio_sinks.len()) as u16,
                )
                .with_latency_control(0)
                .into_bytes(),
        )?;

        for clock in self.controls.iter() {
            clock.write_descriptor(writer);
        }
        for usb_source in self.stream_sources.iter() {
            usb_source.write_descriptor(writer);
        }
        for ext_audio_sink in self.ext_audio_sinks.iter() {
            ext_audio_sink.write_descriptor(writer);
        }
        for usb_sink in self.stream_sinks.iter() {
            usb_sink.write_descriptor(writer);
        }
        for ext_audio_source in self.ext_audio_sources.iter() {
            ext_audio_source.write_descriptor(writer);
        }

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

        for usb_source in self.stream_sources.iter() {
            usb_source.write_stream_iface(writer)?;
        }

        for usb_sink in self.stream_sinks.iter() {
            usb_sink.write_stream_iface(writer)?;
        }

        log::info!("Writer position is {}", writer.position());

        Ok(())
    }

    fn get_alt_setting(&mut self, interface: InterfaceNumber) -> Option<u8> {
        let i: u8 = interface.into();
        log::info!("get_alt_setting(iface=0x{:x})", i);
        if interface == self.stream_sources[0].iface {
            Some(self.alt_setting_2)
        } else if interface == self.stream_sinks[0].iface {
            Some(self.alt_setting_3)
        } else {
            None
        }
    }

    fn set_alt_setting(&mut self, interface: InterfaceNumber, alt: u8) -> bool {
        let i: u8 = interface.into();
        log::info!("set_alt_setting(iface=0x{:x}, alt={})", i, alt);
        if interface == self.stream_sources[0].iface {
            self.alt_setting_2 = alt;
            true
        } else if interface == self.stream_sinks[0].iface {
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

            for control in self.controls.iter() {
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

        if req.recipient == usb_device::control::Recipient::Interface
            && req.request == 11
            && req.index == 1
        {
            xfer.accept().expect("Couldn't accept");
        } else if req.recipient == usb_device::control::Recipient::Interface
            && req.request == 11
            && req.index == 2
        {
            xfer.accept().expect("Couldn't accept");
        } else if req.recipient == usb_device::control::Recipient::Interface
            && req.request_type == usb_device::control::RequestType::Class
            && req.request == 1
            && req.value == 256
            && req.index == 256
        {
            // Setting the Clock Valid control?
            xfer.accept().expect("Couldn't accept");
        }
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        for source in self.stream_sources.iter_mut() {
            if source.endpoint.address().index() == addr.index() {
                source.recv_data();
                break;
            }
        }
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        self.nbytes_sent += 45;
        for sink in self.stream_sinks.iter_mut() {
            if sink.endpoint.address().index() == addr.index() {
                sink.endpoint_in_complete();
                break;
            }
        }
    }
}
