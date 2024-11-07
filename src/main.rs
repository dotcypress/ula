#![no_std]
#![no_main]

extern crate panic_probe;
extern crate rp2040_hal as hal;
extern crate rtic;

mod analyzer;
mod sampler;
mod trigger;

use defmt_rtt as _;

use analyzer::*;
use cortex_m::singleton;
use embedded_hal::digital::OutputPin;
use hal::dma::{self, *};
use hal::gpio::*;
use hal::pac;
use hal::pio::*;
use hal::usb::UsbBus;
use rtic_monotonics::rp2040_timer_monotonic;
use sampler::*;
use trigger::*;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

/// Number of probes available for the analyzer.
pub const PROBES: usize = 16;
/// Memory size allocated for sample storage.
pub const SAMPLE_MEMORY: usize = 200_000;
/// Sampling rate in Hertz.
pub const SAMPLE_RATE: usize = 100_000_000;

/// Base pin number for PIO operations.
pub const PIN_BASE: usize = 0;
/// Frequency of the external crystal oscillator in Hertz.
pub const XTAL_FREQ_HZ: u32 = 12_000_000_u32;

#[cfg(not(feature = "generic-bootloader"))]
#[used]
#[no_mangle]
#[link_section = ".boot2"]
/// Bootloader configuration for non-generic bootloader.
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[cfg(feature = "generic-bootloader")]
#[used]
#[no_mangle]
#[link_section = ".boot2"]
/// Bootloader configuration for generic bootloader.
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

rp2040_timer_monotonic!(Mono);

#[rtic::app(device = pac, peripherals = true)]
mod app {
    use super::*;

    /// Shared resources accessible by multiple tasks.
    #[shared]
    struct Shared {
        /// Instance of the Logic Analyzer.
        analyzer: LogicAnalyzer,
    }

    /// Local resources exclusive to specific tasks.
    #[local]
    struct Local {}

    /// Initialization function executed at startup.
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Reset the SIO to a known state.
        unsafe {
            hal::sio::spinlock_reset();
        }

        let mut resets = ctx.device.RESETS;
        let mut watchdog = hal::Watchdog::new(ctx.device.WATCHDOG);
        // Initialize clocks and PLLs.
        let clocks = hal::clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // Manually set PLL dividers.
        unsafe {
            let pll = &*pac::PLL_SYS::ptr();
            pll.prim()
                .modify(|_, w| w.postdiv1().bits(5).postdiv2().bits(3));
        };

        // Initialize USB peripheral.
        let usb_regs = ctx.device.USBCTRL_REGS;
        let usb_dpram = ctx.device.USBCTRL_DPRAM;
        let usb_bus = UsbBus::new(usb_regs, usb_dpram, clocks.usb_clock, true, &mut resets);
        let usb_bus: &'static UsbBusAllocator<UsbBus> =
            singleton!(: UsbBusAllocator<UsbBus> = UsbBusAllocator::new(usb_bus)).unwrap();

        // Initialize serial port over USB.
        let serial = SerialPort::new(usb_bus);
        let info = StringDescriptors::default()
            .manufacturer("Ferris & Co")
            .product("vitaly.codes/ula")
            .serial_number("_ula_");
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[info])
            .unwrap()
            .device_class(2)
            .build();

        // Enable USB interrupt.
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
        };

        // Split DMA channels.
        let dma = ctx.device.DMA.split(&mut resets);
        // Split PIO0 into individual components.
        let (pio, sm, _, _, _) = ctx.device.PIO0.split(&mut resets);

        // Initialize SIO and configure GPIO pins.
        let sio = hal::Sio::new(ctx.device.SIO);
        let pins = Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // Configure GPIO pins for PIO functionality.
        pins.gpio0.into_function::<FunctionPio0>();
        pins.gpio1.into_function::<FunctionPio0>();
        pins.gpio2.into_function::<FunctionPio0>();
        pins.gpio3.into_function::<FunctionPio0>();
        pins.gpio4.into_function::<FunctionPio0>();
        pins.gpio5.into_function::<FunctionPio0>();
        pins.gpio6.into_function::<FunctionPio0>();
        pins.gpio7.into_function::<FunctionPio0>();
        pins.gpio8.into_function::<FunctionPio0>();
        pins.gpio9.into_function::<FunctionPio0>();
        pins.gpio10.into_function::<FunctionPio0>();
        pins.gpio11.into_function::<FunctionPio0>();
        pins.gpio12.into_function::<FunctionPio0>();
        pins.gpio13.into_function::<FunctionPio0>();
        pins.gpio14.into_function::<FunctionPio0>();
        pins.gpio15.into_function::<FunctionPio0>();

        // Initialize the status LED as a push-pull output.
        let status_led = pins.gpio25.into_push_pull_output();
        // Create a new instance of the Logic Analyzer.
        let analyzer = LogicAnalyzer::new(usb_dev, serial, pio, sm, dma, status_led);

        (Shared { analyzer }, Local {})
    }

    /// Interrupt handler for USB controller events.
    ///
    /// This task is triggered by USB interrupts and polls the Logic Analyzer
    /// for incoming serial commands.
    #[task(binds = USBCTRL_IRQ, shared = [analyzer])]
    fn usb_irq(mut ctx: usb_irq::Context) {
        ctx.shared.analyzer.lock(|analyzer| analyzer.poll_serial());
    }

    /// Interrupt handler for DMA channel 0 events.
    ///
    /// This task is triggered by DMA interrupts and notifies the Logic Analyzer
    /// that data acquisition has been completed.
    #[task(binds = DMA_IRQ_0, shared = [analyzer])]
    fn dma_irq(mut ctx: dma_irq::Context) {
        ctx.shared
            .analyzer
            .lock(|analyzer| analyzer.acquisition_done());
    }
}
