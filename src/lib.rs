#![no_std]

#[macro_use]
extern crate alloc;

use usb_device::class_prelude::*;
use modular_bitfield::prelude::*;

mod descriptors;

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

pub struct UsbAudio<'a, B: usb_device::bus::UsbBus> {
    pub nbytes: usize,
    pub nbytes_sent: usize,
    pub iface: usb_device::class_prelude::InterfaceNumber,
    pub iface2: InterfaceNumber,
    pub iface3: InterfaceNumber,
    pub data_ep: usb_device::endpoint::EndpointOut<'a, B>,
    pub source_ep: usb_device::endpoint::EndpointIn<'a, B>,
}

impl<B: usb_device::bus::UsbBus> UsbAudio<'_, B> {
    pub fn new(alloc: &usb_device::bus::UsbBusAllocator<B>, max_packet_size: u16) -> UsbAudio<'_, B> {
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
            iface2: alloc.interface(),
            iface3: alloc.interface(),
            data_ep: alloc.isochronous(IsochronousSynchronizationType::NoSynchronization, IsochronousUsageType::Data, 200, 1),
            source_ep: alloc.isochronous(IsochronousSynchronizationType::NoSynchronization, IsochronousUsageType::Data, 64, 1),
            //mic_data: data,
        }
    }
}

impl<'a, B: usb_device::bus::UsbBus> usb_device::class::UsbClass<B> for UsbAudio<'a, B> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()> {
        /*
        The design of this audio device is as follows:

        USB IN  <--> Speakers
        USB OUT <--> Feature Unit <--> Microphone
        */

        writer.iad(self.iface, 3, 1 /* AUDIO */, 0 /* CONTROL */, 0x20)?;
        writer.interface(self.iface, 1 /* AUDIO */, 1 /* CONTROL */, 0x20)?;
        let CS_INTERFACE = 0x24;
        // Setup the audio control
        // Clock sources are 8 bytes. Input terminals are 17. Output terminals are 12.
        writer.write(
            CS_INTERFACE,
            &AudioClass::new()
                .with_subtype(Subtype::Header)
                .with_spec_number_major(0)
                .with_spec_number_minor(0)
                .with_category(0xff)
                .with_total_length(8 + 17*2 + 12*2)
                .with_latency_control(0)
                .into_bytes()
        )?;

        // USB IN <--> Speakers route
        // USB IN
        writer.write(CS_INTERFACE,
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
                .into_bytes()
        )?;
        writer.write(
            CS_INTERFACE,
            &ClockSource::new()
                .with_subtype(Subtype::ClockSource)
                .with_clock_id(2)
                .with_clock_type(ClockType::External)
                .with_sync_to_sof(false)
                .with_associated_terminal_id(1)
                .with_name(0)
                .into_bytes()
        )?;

        // Speakers
        writer.write(
            CS_INTERFACE,
            &OutputTerminal::new()
                .with_subtype(Subtype::OutputTerminal)
                .with_terminal_id(3)
                .with_terminal_type(0x0301)
                .with_associated_terminal_id(0)
                .with_source_id(2)
                .with_clock_source_id(2)
                .with_name(0)
                .into_bytes()
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
                .with_source_id(6)
                .with_clock_source_id(2)
                .with_name(0)
                .into_bytes()
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
                .into_bytes()
        )?;

        // Feature unit in Microphone path
        writer.write(
            CS_INTERFACE,
            &[
                6, // FEATURE_UNIT
                6, // Unit ID
                5, // Source ID
                0x0, 0x0, 0x0, 0x0, // bmaControls
                0, // iFeature
            ],
        )?;

        // Setup the speaker streaming
        //writer.iad(self.iface2, 1, 1 /* AUDIO */, 2 /* STREAMING */, 0)?;
        writer.interface(self.iface2, 1 /* AUDIO */, 2 /* STREAMING */, 0x20)?;
        writer.write(
            CS_INTERFACE,
            &[
                1, // AS_GENERAL
                1, // bTerminalLink (terminal ID of output term)
                0, // bmControls
                1, // bFormatType (PCM)
                0x1, 0x0, 0x0, 0x0, // bmFormats (PCM)
                2,   // bNrChannels
                0x3, 0, 0, 0, // bmChannelConfig
                0, // iChannelNames
            ],
        )?;
        writer.write(
            CS_INTERFACE,
            &[
                0x02, // FORMAT_TYPE
                0x01, // FORMAT_TYPE_I
                2, // bSubslotSize (1 byte samples)
                16, // bBitResolution (8 bits in byte used)
            ]
        )?;

        writer.endpoint(&self.data_ep)?;

        // Class specific endpoint descriptor
        writer.write(
            0x25, // CS_ENDPOINT
            &[
                1, // EP_GENERAL
                0, // bmAttributes
                0, // bmControls
                0, // bLockDelayUnits
                0, 0, // wLockDelay
            ]
        )?;

        // Setup the microphone streaming
        writer.interface(self.iface3, 1 /* AUDIO */, 2 /* STREAMING */, 0x20)?;
        writer.write(
            CS_INTERFACE,
            &[
                1, // AS_GENERAL
                4, // bTerminalLink (terminal ID of input term)
                0, // bmControls
                1, // bFormatType (PCM)
                0x1, 0x0, 0x0, 0x0, // bmFormats (PCM)
                1,   // bNrChannels
                0x1, 0, 0, 0, // bmChannelConfig
                0, // iChannelNames
            ],
        )?;
        writer.write(
            CS_INTERFACE,
            &[
                0x02, // FORMAT_TYPE
                0x01, // FORMAT_TYPE_I
                1, // bSubslotSize (1 byte samples)
                8, // bBitResolution (8 bits in byte used)
            ]
        )?;

        writer.endpoint(&self.source_ep)?;

        // Class specific endpoint descriptor
        writer.write(
            0x25, // CS_ENDPOINT
            &[
                1, // EP_GENERAL
                0, // bmAttributes
                0, // bmControls
                0, // bLockDelayUnits
                0, 0, // wLockDelay
            ]
        )?;

        Ok(())
    }

    fn control_in(&mut self, xfer: ControlIn<'_, '_, '_, B>) {
        let req = xfer.request();
        if req.request_type == usb_device::control::RequestType::Class && req.recipient == usb_device::control::Recipient::Interface {
            let reqtype = ControlType::from(req.request);
            let control_selector = req.value >> 8;
            let channel_number = req.value & 0xFF;
            let interface = req.index & 0xFF;
            let entity_id = req.index >> 8;

            //log::info!("control: dir={:?} reqtype={:?} recip={:?} req={} value={} idx={} len={}", req.direction, req.request_type, req.recipient, req.request, req.value, req.index, req.length);
            //if req.recipient == usb_device::control::Recipient::Interface && req.value == 256 && req.request == 2 && req.index == 514 {
            if reqtype == ControlType::Range && entity_id == 2 {
                xfer.accept_with(&[
                    0x1, 0x0, // 1 subrange
                    0x44, 0xac, 0x00, 0x00, // MIN
                    0x44, 0xac, 0x00, 0x00, // MAX
                    0x0, 0x0, 0x0, 0x0, // RES
                ]).expect("Couldn't accept transfer!");
            } else if reqtype == ControlType::Current && entity_id == 2 {
                xfer.accept_with(&[
                    0x44, 0xac, 0x00, 0x00, // CUR
                ]).expect("Couldn't accept transfer!");
            } else if reqtype == ControlType::Range && entity_id == 4 {
                xfer.accept_with(&[
                    0x1, 0x0, // 1 subrange
                    0x44, 0xac, 0x00, 0x00, // MIN
                    0x44, 0xac, 0x00, 0x00, // MAX
                    0x0, 0x0, 0x0, 0x0, // RES
                ]).expect("Couldn't accept transfer!");
            } else if reqtype == ControlType::Current && entity_id == 4 {
                xfer.accept_with(&[
                    0x44, 0xac, 0x00, 0x00, // CUR
                ]).expect("Couldn't accept transfer!");
            } else {
                log::info!("unhandled audio GET: {:?} CS={} chan={} iface={} eid={}", reqtype, control_selector, channel_number, interface, entity_id);
            }
        } else {
            log::info!("control: dir={:?} reqtype={:?} recip={:?} req={} value={} idx={} len={}", req.direction, req.request_type, req.recipient, req.request, req.value, req.index, req.length);
        }
    }

    fn control_out(&mut self, xfer: ControlOut<'_, '_, '_, B>) {
        /*
        bytes recv'd = 1174468
        [INFO audio]: Num bytes recv'd = 1218568
        [INFO audio]: control: dir=Out reqtype=Standard recip=Interface req=11 value=0 idx=2 len=0
        */
        

        let req = xfer.request();
        log::info!("control: dir={:?} reqtype={:?} recip={:?} req={} value={} idx={} len={}", req.direction, req.request_type, req.recipient, req.request, req.value, req.index, req.length);
        /*if req.request_type == usb_device::control::RequestType::Class && req.recipient == usb_device::control::Recipient::Interface {
            let reqtype = ControlType::from(req.request);
            let control_selector = req.value >> 8;
            let channel_number = req.value & 0xFF;
            let interface = req.index & 0xFF;
            let entity_id = req.index >> 8;

            log::info!("audio SET: {:?} CS={} chan={} iface={} eid={}", reqtype, control_selector, channel_number, interface, entity_id);*/
            //log::info!("control: dir={:?} reqtype={:?} recip={:?} req={} value={} idx={} len={}", req.direction, req.request_type, req.recipient, req.request, req.value, req.index, req.length);
        if req.recipient == usb_device::control::Recipient::Interface && req.request == 11 && req.index == 1 {
            //log::info!("Handling");
            //xfer.reject().expect("Couldn't reject");
            xfer.accept().expect("Couldn't accept");
        } else if req.recipient == usb_device::control::Recipient::Interface && req.request == 11 && req.index == 2 {
            //log::info!("Handling");
            //xfer.reject().expect("Couldn't reject");
            xfer.accept().expect("Couldn't accept");
        }/* else if req.recipient == usb_device::control::Recipient::Endpoint && req.request == 1 && req.index == 1 {
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
                Ok(x) => {
                    x
                },
                Err(e) => {
                    log::error!("Received error {:?}", e);
                    return;
                }
            };
            //log::info!("RX'd data: EP{}", addr.index());
            //log::info!("{}", buf[5]);
            self.nbytes += nbytes;
        }
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        //if addr.index() == 1 {
            //self.cnt += 1;
            //self.data_ep.write(&[5; 64]);
        self.nbytes_sent += 100;//self.mic_data.len();
        //self.source_ep.write(&self.mic_data);
        //}
    }
}
