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

struct AdcSource {
    regs: ral::adc::Instance,
}

impl AdcSource {
    fn new(regs: ral::adc::Instance) -> AdcSource {
        AdcSource {
            regs
        }
    }
}

impl hal::dma::Source<u16> for AdcSource {
    type Error = ();

    const SOURCE_REQUEST_SIGNAL: u32 = 24;

    /// Returns a pointer to the register from which the DMA channel
    /// reads data
    ///
    /// This is the register that software reads to acquire data from
    /// a device. The type of the pointer describes the type of reads
    /// the DMA channel performs when transferring data.
    ///
    /// This memory is assumed to be static.
    fn source(&self) -> *const u16 {
        //self.regs.R0
        // TODO: Don't hardcode
        0x400c_4024 as *const u16
    }

    /// Perform any actions necessary to enable DMA transfers
    ///
    /// Callers use this method to put the peripheral in a state where
    /// it can supply the DMA channel with data.
    fn enable_source(&mut self) -> Result<(), Self::Error> {
        //let channel = <P as Pin<ADCx>>::Input::U32;
        let channel = 1; // GPIO_AD_B0_12
        ral::modify_reg!(ral::adc, self.regs, HC0, |_| channel);
        Ok(())
    }

    /// Perform any actions necessary to disable or cancel DMA transfers
    ///
    /// This may include undoing the actions in `enable_source()`.
    fn disable_source(&mut self) {
        //
    }

}

static ADC_BUFFER: hal::dma::Buffer<[u16; 256]> = hal::dma::Buffer::new([0; 256]);
static ADC_BUFFER2: hal::dma::Buffer<[u16; 256]> = hal::dma::Buffer::new([0; 256]);

static mut adc_count: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

static mut global_dma_periph: Option<hal::dma::Peripheral<AdcSource, u16, hal::dma::Linear<u16>, hal::dma::Linear<u16>>> = None;
static mut off_adc_buffer: Option<hal::dma::Linear<u16>> = None;

#[interrupt]
fn DMA8_DMA24() {
    unsafe {
        if let Some(periph) = &mut global_dma_periph {
            if periph.is_receive_interrupt() {
                periph.receive_clear_interrupt();
            }

            let mut rx_buffer = periph.receive_complete().unwrap();
            //adc_samps += 256;
            periph.start_receive(off_adc_buffer.take().unwrap()).unwrap();
            off_adc_buffer = Some(rx_buffer);
            adc_count.fetch_add(256, core::sync::atomic::Ordering::SeqCst);
        }
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
    // Clocking
    // Use the IPG clock
    log::info!("IPG clock is set to {:?}", ipg_hz);

    let adc1 = adc1.into_regs();

    hal::ral::write_reg!(hal::ral::adc, adc1, CFG,
        OVWREN: 1,
        AVGS: 0b00, ADTRG: 0, REFSEL: 0, ADHSC: 0, ADSTS: 0b11,
        ADLPC: 0, ADIV: 0b11, ADLSMP: 1, MODE: 0b10, ADICLK: 0b00);
    hal::ral::write_reg!(hal::ral::adc, adc1, GC, ADCO: 1, AVGE: 0, DMAEN: 1);

    // Setup DMA for the ADC
    let mut channel = dma_channels[8].take().unwrap();
    channel.set_interrupt_on_completion(true);

    //let mut adc_buffer = hal::dma::Circular::new(&ADC_BUFFER).unwrap();
    let mut adc_buffer = hal::dma::Linear::new(&ADC_BUFFER).unwrap();
    adc_buffer.set_transfer_len(256);

    let mut off_adc_buffer2 = hal::dma::Linear::new(&ADC_BUFFER2).unwrap();
    off_adc_buffer2.set_transfer_len(256);

    let adc_source = AdcSource::new(adc1);
    let mut dma_peripheral = hal::dma::receive_u16(adc_source, channel);
    dma_peripheral.start_receive(adc_buffer).unwrap();

    unsafe {
        global_dma_periph = Some(dma_peripheral);
        off_adc_buffer = Some(off_adc_buffer2);
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

    let mut n_sent = 1;

    let mut last_sent_bytes = 0;
    let mut loops = 0;
    let mut update_countdown = 0;
    let mut adc_min = 0;
    let mut adc_max = 0;
    let mut last_adc = 0;
    let mut mic_data = vec!();
    loop {
        loops += 1;
        time_elapse(&mut gpt1, || {
            let adc_samps = unsafe { adc_count.load(core::sync::atomic::Ordering::SeqCst) };
            log::info!("{} RX'd = {} TX'd = {} loops = {} ADC samps = {} min/max={}/{}", gpt2.count(), audio.nbytes, audio.nbytes_sent, loops, adc_samps, adc_min, adc_max);
            unsafe {
                adc_count.store(0, core::sync::atomic::Ordering::SeqCst);
            }

            adc_min = last_adc;
            adc_max = last_adc;
            led.toggle()
        });
        imxrt_uart_log::dma::poll();
        if !device.poll(&mut [&mut audio]) {
            //continue;
        }

        if mic_data.len() == 100 {
            audio.source_ep.write(&mic_data[..]);
            mic_data = vec!();
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
