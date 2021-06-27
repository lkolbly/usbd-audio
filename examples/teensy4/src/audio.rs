//! Demonstrate a USB serial device
//!
//! Flash your Teensy 4 with this example. Then, connect a serial
//! interface to the USB device. You should see all inputs echoed
//! back to you.
//!
//! This example also supports debug logs over UART2, using pins
//! 14 and 15.

#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

#[macro_use]
extern crate alloc;

mod pfb;
use pfb::PolyphaseFilterBank;

use cortex_m_rt::interrupt;
use embedded_hal::adc::OneShot;
use support::bsp::interrupt;
use support::bsp::t41;
use support::hal;
use support::hal::adc;

use alloc_cortex_m::CortexMHeap;
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_audio::UsbAudio;

const UART_BAUD: u32 = 115_200;
const GPT_OCR: hal::gpt::OutputCompareRegister = hal::gpt::OutputCompareRegister::One;
const BLINK_PERIOD: core::time::Duration = core::time::Duration::from_millis(500);

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
static mut HEAP: [u8; 128 * 1024] = [0; 128 * 1024];

#[alloc_error_handler]
fn handle_alloc_error(allocation: core::alloc::Layout) -> ! {
    log::error!("Failing allocation: {:?}", allocation);
    panic!("Allocation failed!");
}

static ADC_BUFFER: hal::dma::Buffer<[u16; 256]> = hal::dma::Buffer::new([0; 256]);
static ADC_BUFFER2: hal::dma::Buffer<[u16; 256]> = hal::dma::Buffer::new([0; 256]);
static mut streaming_adc: Option<hal::adc::StreamingAdc<hal::iomuxc::adc::ADC1, t41::P24>> = None;

#[interrupt]
fn DMA8_DMA24() {
    unsafe {
        if let Some(adc) = &mut streaming_adc {
            adc.handle_interrupt();
        }
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, 128 * 1024) }

    let hal::Peripherals {
        iomuxc,
        mut ccm,
        dma,
        uart,
        mut dcdc,
        gpt1,
        gpt2,
        adc,
        sai1,
        ..
    } = hal::Peripherals::take().unwrap();
    let sai = sai1;
    let pins = t41::into_pins(iomuxc);
    let mut led = support::configure_led(pins.p13);

    // Timer for blinking
    let (_, ipg_hz) = ccm
        .pll1
        .set_arm_clock(hal::ccm::PLL1::ARM_HZ, &mut ccm.handle, &mut dcdc);

    let mut cfg = ccm.perclk.configure(
        &mut ccm.handle,
        hal::ccm::perclk::PODF::DIVIDE_3,
        hal::ccm::perclk::CLKSEL::IPG(ipg_hz),
    );

    let mut gpt1 = gpt1.clock(&mut cfg);

    gpt1.set_wait_mode_enable(true);
    gpt1.set_mode(hal::gpt::Mode::Reset);
    gpt1.set_enable(true);

    gpt1.set_output_compare_duration(GPT_OCR, BLINK_PERIOD);

    let mut gpt2 = gpt2.clock(&mut cfg);

    gpt2.set_mode(hal::gpt::Mode::FreeRunning);
    gpt2.set_enable(true);

    // DMA initialization (for logging)
    let mut dma_channels = dma.clock(&mut ccm.handle);
    let mut channel = dma_channels[7].take().unwrap();
    channel.set_interrupt_on_completion(false); // We'll poll the logger ourselves...

    //
    // UART initialization (for logging)
    //
    let uarts = uart.clock(
        &mut ccm.handle,
        hal::ccm::uart::ClockSelect::OSC,
        hal::ccm::uart::PrescalarSelect::DIVIDE_1,
    );
    let uart = uarts.uart6.init(pins.p1, pins.p0, UART_BAUD).unwrap();

    let (tx, _) = uart.split();
    imxrt_uart_log::dma::init(
        tx,
        channel,
        imxrt_uart_log::LoggingConfig {
            max_level: log::LevelFilter::Debug,
            filters: &[],
        },
    )
    .unwrap();

    // Setup the SAI mclk
    // Copied from https://github.com/PaulStoffregen/Audio/blob/master/output_i2s.cpp
    let fs = 44_100; //AUDIO_SAMPLE_RATE_EXACT
                     // PLL between 27*24 = 648MHz und 54*24=1296MHz
    let n1 = 4; //SAI prescaler 4 => (n1*n2) = multiple of 4
    let n2 = 1 + (24000000 * 27) / (fs * 256 * n1); // Ends up being 15
    ccm.pll4
        .set(&mut ccm.handle, fs as f64 * 256. * n1 as f64 * n2 as f64);
    unsafe {
        let iomuxc_gpr = hal::ral::iomuxc_gpr::IOMUXC_GPR::steal();
        hal::ral::modify_reg!(hal::ral::iomuxc_gpr, iomuxc_gpr, GPR1,
            SAI1_MCLK_DIR: 1,  // Is output
            SAI1_MCLK1_SEL: 3 // ssi1_clk_root
        );
    }

    // Setup the SAI interface
    let sai1_builder = sai.clock(&mut ccm.handle, (&ccm.pll4).into(), n1 * n2);

    let mut sai1 = sai1_builder.build_1bit_tx(pins.p23, pins.p26, pins.p27, pins.p39);
    sai1.configure_bit_clock(4, false, hal::sai::BitClockSource::Mclk1);
    sai1.set_enabled_channels(&[0]);
    sai1.configure_frame(2, 32, 32, true);
    sai1.configure_frame_sync(false, false, false, true, 32);
    sai1.set_tx_enable(true);

    // ADC initialization
    let (adc1_builder, _) = adc.clock(&mut ccm.handle);

    let mut adc1 = adc1_builder.build(adc::ClockSelect::default(), adc::ClockDivision::default());
    let mut a1 = adc::AnalogInput::new(pins.p24);

    let reading: u16 = adc1.read(&mut a1).unwrap();
    log::info!("Initial ADC reading: {}", reading);

    // Back to UART stuff
    let (ccm, _) = ccm.handle.raw();
    hal::ral::modify_reg!(hal::ral::ccm, ccm, CCGR6, CG1: 0b11, CG0: 0b11);

    // Setup the ADC
    log::info!("IPG clock is set to {:?}", ipg_hz);

    hal::ral::write_reg!(hal::ral::adc, adc1.get_regs(), CFG,
        OVWREN: 1,
        AVGS: 0b00, ADTRG: 0, REFSEL: 0, ADHSC: 0, ADSTS: 0b11,
        ADLPC: 0, ADIV: 0b11, ADLSMP: 1, MODE: 0b10, ADICLK: 0b00);

    let mut channel = dma_channels[8].take().unwrap();
    channel.set_interrupt_on_completion(true);
    unsafe {
        streaming_adc = Some(hal::adc::StreamingAdc::new(
            adc1,
            a1,
            channel,
            &ADC_BUFFER,
            &ADC_BUFFER2,
        ));
    }

    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::DMA8_DMA24);
    }

    // USB initialization
    let bus_adapter = support::new_bus_adapter();
    let bus = usb_device::bus::UsbBusAllocator::new(bus_adapter);

    let audio_allocator = usbd_audio::EntityAllocator::new(&bus);

    let mut speaker_clock = audio_allocator.make_clock();
    let speaker_usb_source = audio_allocator.make_usb_stream_source(
        &bus,
        &speaker_clock,
        usbd_audio::ChannelSet::new()
            .with_spatial(usbd_audio::SpatialChannel::FrontLeft)
            .with_spatial(usbd_audio::SpatialChannel::FrontRight),
    );
    let speaker_sink =
        audio_allocator.make_external_audio_sink(&speaker_clock, &speaker_usb_source);

    let speaker_usb_source2 = audio_allocator.make_usb_stream_source(
        &bus,
        &speaker_clock,
        usbd_audio::ChannelSet::new()
            .with_spatial(usbd_audio::SpatialChannel::FrontLeft)
            .with_spatial(usbd_audio::SpatialChannel::FrontRight),
    );
    let speaker_sink2 =
        audio_allocator.make_external_audio_sink(&speaker_clock, &speaker_usb_source2);

    let mut mic_clock = audio_allocator.make_clock();
    let mic_source = audio_allocator.make_external_audio_source(
        &mic_clock,
        usbd_audio::ChannelSet::new().with_spatial(usbd_audio::SpatialChannel::FrontLeft),
    );
    let mic_usb_sink = audio_allocator.make_usb_stream_sink(&bus, &mic_source, &mic_clock);

    let controls: [&mut dyn usbd_audio::ControlEntity<_>; 2] = [&mut speaker_clock, &mut mic_clock];
    let mut input_streams = [speaker_usb_source, speaker_usb_source2];
    let mut output_streams = [mic_usb_sink];
    let ext_sinks = [speaker_sink, speaker_sink2];
    let ext_sources = [mic_source];
    let mut audio = UsbAudio::new(
        &audio_allocator,
        &controls[..],
        &mut input_streams,
        &mut output_streams,
        &ext_sinks,
        &ext_sources,
    );

    let mut serial = usbd_serial::SerialPort::new(&bus);

    let mut device = UsbDeviceBuilder::new(&bus, UsbVidPid(0x5824, 0x27dd))
        .product("imxrt-usbd")
        .max_packet_size_0(64)
        .device_class(0x0)
        .composite_with_iads()
        .build();

    log::info!("Starting to poll USB device...");

    loop {
        imxrt_uart_log::dma::poll();
        if !device.poll(&mut [&mut audio, &mut serial]) {
            continue;
        }
        let state = device.state();
        if state == usb_device::device::UsbDeviceState::Configured {
            break;
        }
    }

    device.bus().configure();
    led.set();

    unsafe {
        streaming_adc.as_mut().unwrap().start();
    }

    let mut pfb = PolyphaseFilterBank::new();

    log::info!("SAI version: {:?}", sai1.version());
    log::info!(
        "0x{:x}",
        hal::ral::read_reg!(hal::ral::sai, sai1.regs(), TCSR)
    );

    let mut loops = 0;
    let mut adc_min = 0;
    let mut adc_max = 0;
    let mut filtered_adc_min: f32 = 0.0;
    let mut filtered_adc_max: f32 = 0.0;
    let mut nsamps = 0;
    let mut filtered_samps = 0;
    let mut last_processing_time = 0;
    let mut blocks = 0;
    let mut samps_dropped = 0;
    let mut i2s_samps_sent = 0;
    let mut i2s_underruns = 0;
    let mut i2s_usb_overruns = 0;
    let mut max_out_sample = 0;
    let mut fifo_size_hist: [u32; 32] = [0; 32];
    loop {
        loops += 1;

        {
            let sadc = unsafe { streaming_adc.as_mut().unwrap() };
            sadc.poll(|buf| {
                let start = gpt2.count();
                adc_min = *buf.iter().chain(core::iter::once(&adc_min)).min().unwrap();
                adc_max = *buf.iter().chain(core::iter::once(&adc_max)).max().unwrap();
                nsamps += buf.len();
                let mut readings = vec![];
                for i in 0..buf.len() {
                    let reading: u8 = (buf[i] >> 4) as u8;
                    let reading: i8 = reading.wrapping_sub(128) as i8;
                    readings.push(reading as f32);
                }

                for samp in pfb.consume(&readings[..]).iter() {
                    if *samp < filtered_adc_min {
                        filtered_adc_min = *samp;
                    }
                    if *samp > filtered_adc_max {
                        filtered_adc_max = *samp;
                    }
                    if audio.stream_sinks[0].samps.len() < 1024 {
                        let x = *samp * 10.0;
                        audio.stream_sinks[0].samps.push(x as i8 as u8);
                    } else {
                        samps_dropped += 1;
                    }
                    filtered_samps += 1;
                }

                let end = gpt2.count();
                if end < start {
                    last_processing_time = 0xffff_ffff - (start - end);
                } else {
                    last_processing_time = end - start;
                }
            });
        }

        time_elapse(&mut gpt1, || {
            // Note: GPT2 increments at 25M ticks per second (40ns/tick)

            let min_fifo_size = fifo_size_hist
                .iter()
                .enumerate()
                .filter(|(_, count)| **count > 0)
                .map(|(idx, _)| idx)
                .next()
                .unwrap_or(0);
            let max_fifo_size = fifo_size_hist
                .iter()
                .enumerate()
                .rev()
                .filter(|(_, count)| **count > 0)
                .map(|(idx, _)| idx)
                .next()
                .unwrap_or(0);
            let median_fifo_size = fifo_size_hist
                .iter()
                .enumerate()
                .max_by_key(|(_, count)| *count)
                .map(|(idx, _)| idx)
                .unwrap_or(0);

            log::info!(
                "{} RX'd = {} TX'd = {} blocks = {} dropped = {} loops = {} proctime = {} nsamps = {} filtered_samps = {} min/max={}/{} filtered range={}/{} I2S samps sent = {} underruns = {} overruns = {} max = {}",
                gpt2.count(), audio.nbytes, audio.nbytes_sent, blocks, samps_dropped, loops, last_processing_time, nsamps, filtered_samps, adc_min, adc_max, filtered_adc_min, filtered_adc_max, i2s_samps_sent, i2s_underruns, audio.usb_overruns, max_out_sample,
            );

            log::info!("{}/{}/{}", min_fifo_size, median_fifo_size, max_fifo_size);
            let mut max_fifo_count = *fifo_size_hist.iter().max().unwrap_or(&0);
            while max_fifo_count > 70 {
                max_fifo_count >>= 1;
                for i in 0..32 {
                    if fifo_size_hist[i] > 1 {
                        fifo_size_hist[i] >>= 1;
                    }
                }
            }
            for i in 0..32 {
                log::info!("{:-<1$}", fifo_size_hist[i], fifo_size_hist[i] as usize);
            }
            fifo_size_hist = [0; 32];

            log::info!(
                "0x{:x}",
                hal::ral::read_reg!(hal::ral::sai, sai1.regs(), TCSR)
            );
            nsamps = 0;
            filtered_samps = 0;
            filtered_adc_min = 0.0;
            filtered_adc_max = 0.0;
            blocks = 0;
            samps_dropped = 0;
            i2s_samps_sent = 0;
            i2s_underruns = 0;
            i2s_usb_overruns = 0;
            audio.usb_overruns = 0;
            max_out_sample = 0;

            match serial.write(b"Hello, world!\r\n") {
                Ok(_count) => {
                    //
                }
                Err(UsbError::WouldBlock) => {
                    //
                }
                Err(e) => {
                    log::error!("Error! {:?}", e);
                }
            }

            adc_min = 2048;
            adc_max = 2048;
            led.toggle()
        });
        imxrt_uart_log::dma::poll();
        if !device.poll(&mut [&mut audio, &mut serial]) {
            //continue;
        }
        audio.poll();

        let mut buf = [0u8; 64];
        match serial.read(&mut buf[..]) {
            Ok(count) => {
                log::info!("Got {} bytes from serial", count);
            }
            Err(UsbError::WouldBlock) => {
                // Buffers full
            }
            Err(err) => {
                log::error!("Got serial error {:?}", err);
            }
        }

        // Update the FIFO status histogram
        {
            let fifo_status = sai1.fifo_status();
            fifo_size_hist[fifo_status.count as usize] += 1;
        }

        // Add data to the I2S FIFO
        loop {
            let fifo_status = sai1.fifo_status();
            if fifo_status.error {
                sai1.clear_fifo_error();
            }

            if fifo_status.count < 30 {
                let has_primary_sample = audio.stream_sources[0].samps.available() > 0;
                let has_secondary_sample = audio.stream_sources[1].samps.available() > 0;

                let mut extract_samp = |idx: usize| {
                    let mut left = 0.;
                    let mut right = 0.;
                    audio.stream_sources[idx].samps.next(|sample: &[u16]| {
                        left = sample[0] as i16 as f32;
                        right = sample[1] as i16 as f32;
                    });
                    (left, right)
                };

                if fifo_status.count < 15 && (has_primary_sample || has_secondary_sample) {
                    let (left, right) = extract_samp(0);
                    let (left2, right2) = extract_samp(1);
                    let left = (left + left2) as i32 as u32;
                    let right = (right + right2) as i32 as u32;
                    sai1.push_sample(left << 16);
                    sai1.push_sample(right << 16);
                    i2s_samps_sent += 1;
                } else if fifo_status.count < 30 && has_primary_sample && has_secondary_sample {
                    let (left, right) = extract_samp(0);
                    let (left2, right2) = extract_samp(1);
                    let left = (left + left2) as i32 as u32;
                    let right = (right + right2) as i32 as u32;
                    sai1.push_sample(left << 16);
                    sai1.push_sample(right << 16);
                    i2s_samps_sent += 1;
                } else {
                    i2s_underruns += 2;
                    break;
                }
            }
            if fifo_status.full {
                break;
            }
        }
    }
}

fn time_elapse(gpt: &mut hal::gpt::GPT, func: impl FnOnce()) {
    let mut status = gpt.output_compare_status(GPT_OCR);
    if status.is_set() {
        status.clear();
        func();
    }
}
