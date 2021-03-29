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

use support::bsp::t41;
use support::hal;
use support::ral;
use support::hal::adc;
use embedded_hal::adc::OneShot;
use support::bsp::interrupt;
use cortex_m_rt::interrupt;

use usb_device::prelude::*;
use usb_device::class_prelude::*;
use alloc_cortex_m::CortexMHeap;
use micromath::F32Ext;
use usbd_audio::UsbAudio;

const UART_BAUD: u32 = 115_200;
const GPT_OCR: hal::gpt::OutputCompareRegister = hal::gpt::OutputCompareRegister::One;
const BLINK_PERIOD: core::time::Duration = core::time::Duration::from_millis(500);

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
static mut HEAP: [u8; 128*1024] = [0; 128*1024];

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

/// The ADC samples at 93696Hz, we want to output at 44100Hz
/// So we need to resample at 1/2.1246, which we'll approximate with
/// 1000/2125 or 8/17 (dividing out 125), giving us a true rate of ~44092Hz
///
/// For interpolation, we insert 7 zeroes after every sample, then we
/// filter everything above Fs/16. The resulting sample rate Fsi = 749,568Hz
///
/// For decimation, we filter out everything above 20kHz (Fsi/38), and take
/// every 17th sample.
///
/// For each output sample, we skip ahead 2 1/8th samples. Thus, with
/// 8 filters, we consume 17 samples (2*8+8/8) and output 8 samples.
///
/// In general terms, with an N-tap filter M[0..N], output O[i] is:
/// O[i] = sum(V[i-N+x] * M[x] for x in 0..N)
/// V[i] = I[i/17] if i%17 == 0
///      = 0 otherwise
///
/// Therefore, with N=50:
/// O[0] = V[-50] * M[0] + V[-33] * M[17] + V[-16] * M[34]
/// O[1] = ?
struct PolyphaseFilterBank {
    sample_offset: usize,
    nsamps: usize,
    raw_samples: [f32; 50],
}

const FILTER: [f32; 50] = [-1.5571565755041362e-20,5.299655672641262e-06,3.729570824646748e-05,0.00012457763600283632,0.00030323871904505245,0.0006182758531686576,0.0011238702437696656,0.0018821463770603017,0.0029601834785402945,0.004425282087467552,0.006338740597132057,0.008748640145754124,0.011682338016292702,0.015139500358883382,0.019086542908262744,0.02345328251739534,0.028132434417186106,0.03298233459649817,0.03783294946319836,0.04249489045854946,0.046770818593343866,0.050468341909227984,0.0534133117654359,0.055462336662509264,0.05651336783135692,0.05651336783135692,0.05546233666250927,0.05341331176543591,0.050468341909227984,0.04677081859334388,0.04249489045854947,0.03783294946319837,0.032982334596498186,0.028132434417186106,0.023453282517395355,0.019086542908262744,0.015139500358883395,0.011682338016292707,0.008748640145754133,0.006338740597132062,0.00442528208746756,0.002960183478540299,0.0018821463770603017,0.0011238702437696673,0.0006182758531686576,0.0003032387190450536,0.0001245776360028364,3.7295708246467734e-05,5.299655672641164e-06,-1.5571565755041362e-20];

impl PolyphaseFilterBank {
    fn consume(&mut self, data: &[f32]) -> alloc::vec::Vec<f32> {
        let mut result = vec![];
        data.iter().for_each(|sample| {
            self.raw_samples[(self.sample_offset + self.nsamps) % 50] = *sample;
            self.nsamps = (self.nsamps + 1) % 50;
            while self.nsamps >= 21 {
                let mut i = [0.0; 21];
                for idx in 0..21 {
                    i[idx] = self.raw_samples[(self.sample_offset + idx) % 50];
                }
                self.nsamps -= 17;
                self.sample_offset = (self.sample_offset + 17) % 50;
                let chunk = self.compute(&i);
                for elem in chunk.iter() {
                    result.push(*elem);
                }
            }
        });
        result
    }

    fn compute(&self, I: &[f32; 21]) -> [f32; 8] {
        let F = &FILTER;
        let mut O = [0.0; 8];
        O[0] = I[0] * F[7] + I[1] * F[15] + I[2] * F[23] + I[3] * F[31] + I[4] * F[39] + I[5] * F[47];
        O[1] = I[2] * F[6] + I[3] * F[14] + I[4] * F[22] + I[5] * F[30] + I[6] * F[38] + I[7] * F[46];
        O[2] = I[4] * F[5] + I[5] * F[13] + I[6] * F[21] + I[7] * F[29] + I[8] * F[37] + I[9] * F[45];
        O[3] = I[6] * F[4] + I[7] * F[12] + I[8] * F[20] + I[9] * F[28] + I[10] * F[36] + I[11] * F[44];
        O[4] = I[8] * F[3] + I[9] * F[11] + I[10] * F[19] + I[11] * F[27] + I[12] * F[35] + I[13] * F[43];
        O[5] = I[10] * F[2] + I[11] * F[10] + I[12] * F[18] + I[13] * F[26] + I[14] * F[34] + I[15] * F[42];
        O[6] = I[12] * F[1] + I[13] * F[9] + I[14] * F[17] + I[15] * F[25] + I[16] * F[33] + I[17] * F[41] + I[18] * F[49];
        O[7] = I[14] * F[0] + I[15] * F[8] + I[16] * F[16] + I[17] * F[24] + I[18] * F[32] + I[19] * F[40] + I[20] * F[48];
        O
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, 128*1024) }

    let hal::Peripherals {
        iomuxc,
        mut ccm,
        dma,
        uart,
        mut dcdc,
        gpt1,
        gpt2,
        adc,
        ..
    } = hal::Peripherals::take().unwrap();
    let pins = t41::into_pins(iomuxc);
    let mut led = support::configure_led(pins.p13);

    // Timer for blinking
    let (_, ipg_hz) =
        ccm.pll1
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
    imxrt_uart_log::dma::init(tx, channel, imxrt_uart_log::LoggingConfig {
        max_level: log::LevelFilter::Debug,
        filters: &[
        ],
    }).unwrap();

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
        streaming_adc = Some(hal::adc::StreamingAdc::new(adc1, a1, channel, &ADC_BUFFER, &ADC_BUFFER2));
    }

    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::DMA8_DMA24);
    }

    // USB initialization
    let bus_adapter = support::new_bus_adapter();
    let bus = usb_device::bus::UsbBusAllocator::new(bus_adapter);

    let clocks = [usbd_audio::ClockSource::new(2)];
    let mut audio = UsbAudio::new(&bus, 768, &clocks);
    let mut device = UsbDeviceBuilder::new(&bus, UsbVidPid(0x5824, 0x27dd))
        .product("imxrt-usbd")
        .max_packet_size_0(64)
        .device_class(0x0)
        .composite_with_iads()
        .build();

    log::info!("Starting to poll USB device...");

    loop {
        imxrt_uart_log::dma::poll();
        if !device.poll(&mut [&mut audio]) {
            continue;
        }
        let state = device.state();
        if state == usb_device::device::UsbDeviceState::Configured {
            break;
        }
    }

    device.bus().configure();
    led.set();
    /*let mut data = [0; 100];
    for i in 0..100 {
        let x = (i as f32).sin() * 127.;
        let x = x as i8;
        data[i] = x as u8;
    }*/

    unsafe {
        streaming_adc.as_mut().unwrap().start();
    }

    let mut pfb = PolyphaseFilterBank{
        sample_offset: 0,
        nsamps: 0,
        raw_samples: [0.0; 50],
    };

    let mut n_sent = 1;

    let mut last_sent_bytes = 0;
    let mut loops = 0;
    let mut update_countdown = 0;
    let mut adc_min = 0;
    let mut adc_max = 0;
    let mut filtered_adc_min: f32 = 0.0;
    let mut filtered_adc_max: f32 = 0.0;
    let mut last_adc = 0;
    let mut nsamps = 0;
    let mut filtered_samps = 0;
    let mut last_processing_time = 0;
    let mut mic_data = vec!();
    let mut blocks = 0;
    let mut samps_dropped = 0;
    loop {
        loops += 1;

        {
            let mut sadc = unsafe { streaming_adc.as_mut().unwrap() };
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
                    if mic_data.len() < 1024 {
                        let x = *samp * 10.0;
                        mic_data.push(x as i8 as u8);
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
            log::info!(
                "{} RX'd = {} TX'd = {} blocks = {} dropped = {} loops = {} proctime = {} nsamps = {} filtered_samps = {} min/max={}/{} filtered range={}/{}",
                gpt2.count(), audio.nbytes, audio.nbytes_sent, blocks, samps_dropped, loops, last_processing_time, nsamps, filtered_samps, adc_min, adc_max, filtered_adc_min, filtered_adc_max
            );
            nsamps = 0;
            filtered_samps = 0;
            filtered_adc_min = 0.0;
            filtered_adc_max = 0.0;
            blocks = 0;
            samps_dropped = 0;

            adc_min = 2048;
            adc_max = 2048;
            led.toggle()
        });
        imxrt_uart_log::dma::poll();
        if !device.poll(&mut [&mut audio]) {
            //continue;
        }

        if mic_data.len() > 64 {
            let end = if mic_data.len() > 64 { 64 } else { mic_data.len() };
            match audio.source_ep.write(&mic_data[..end]) {
                Ok(_) => {
                    mic_data = mic_data[end..].to_vec();
                },
                Err(usb_device::UsbError::WouldBlock) => {
                    // Would block, do nothing
                    blocks += 1;
                },
                Err(e) => {
                    panic!("Endpoint send error {:?}", e);
                }
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
