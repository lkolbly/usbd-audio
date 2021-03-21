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

    let (ccm, _) = ccm.handle.raw();
    hal::ral::modify_reg!(hal::ral::ccm, ccm, CCGR6, CG1: 0b11, CG0: 0b11);

    let bus_adapter = support::new_bus_adapter();
    let bus = usb_device::bus::UsbBusAllocator::new(bus_adapter);

    let mut audio = UsbAudio::new(&bus, 768);
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

    let mut n_sent = 1;

    let mut last_sent_bytes = 0;
    let mut loops = 0;
    let mut update_countdown = 0;
    loop {
        loops += 1;
        time_elapse(&mut gpt1, || {
            log::info!("Num bytes recv'd = {} sent = {} loops = {}", audio.nbytes, audio.nbytes_sent, loops);
            led.toggle()
        });
        imxrt_uart_log::dma::poll();
        if !device.poll(&mut [&mut audio]) {
            //continue;
        }

        if update_countdown == 0 {
            let buf = [5u8; 100];
            audio.source_ep.write(&buf);
            //audio.source_ep.write(&audio.mic_data[0..64]);

            // We loop ~526,656 times per second (ish)
            // We want to hit 44,100 samples per second
            // which is one sample per ~11.9423 iterations
            // We send 100 samples at a time
            // so we want to send a packet every 1194 iterations
            // But my math is wrong so let's try this number by trial-and-error
            update_countdown = 2000;
        } else {
            update_countdown -= 1;
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
