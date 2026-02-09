//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

mod bus;
mod devices;
mod oled_demo;

use embassy_executor::Executor;
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::Peri;
use embassy_time::Timer;

use defmt::{error, info, unwrap};
use rp235x_hal::{self as hal};

use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::I2C1;
use embassy_rp::i2c::{self, I2c, Config as I2cConfig, InterruptHandler as I2cInterruptHandler};

use embassy_rp::gpio::{Level, Output};
use embedded_hal_bus::i2c::CriticalSectionDevice;
use embedded_hal::i2c::I2c as _; 

bind_interrupts!(struct Irqs {
    I2C1_IRQ => I2cInterruptHandler<I2C1>;
});

use cortex_m::asm;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use portable_atomic::{AtomicU8, Ordering};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

// Core1 stack
static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<embassy_executor::Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<embassy_executor::Executor> = StaticCell::new();

// 診断用：Core1起動確認フラグ
static CORE1_ALIVE: AtomicU8 = AtomicU8::new(0);

// 診断用LED点滅周期（ms）
static DELAY_MS: AtomicU8 = AtomicU8::new(100);

// 進捗表示用ステージ（USBログが取れない場合の切り分け用）
// LEDは「ステージ回数だけ点滅 → 少し待つ」を繰り返します。
// 250/251 はエラー（高速点滅）。
static STAGE: AtomicU8 = AtomicU8::new(0);

// XIAO系のユーザーLEDは「アクティブLow」（Lowで点灯）のことがあります。
// もしLEDが「光りっぱなし/全く変化しない」場合は、ここを true にして再確認してください。
const LED_ACTIVE_LOW: bool = false;

fn led_on(led: &mut Output<'static>) {
    if LED_ACTIVE_LOW {
        led.set_low();
    } else {
        led.set_high();
    }
}

fn led_off(led: &mut Output<'static>) {
    if LED_ACTIVE_LOW {
        led.set_high();
    } else {
        led.set_low();
    }
}

fn delay_ms_blocking(ms: u32) {
    // だいたい。動作確認用（正確さ不要）
    // 150MHz想定で 150_000 cycles/ms くらい。
    asm::delay(ms.saturating_mul(150_000));
}


enum Fatal {
    ProbeFail,
    InitFail,
    FlushFail,
}

fn fatal_blink_forever(led: &mut Output<'static>, stage: u8, kind: Fatal) -> ! {
    STAGE.store(stage, Ordering::Relaxed);
    loop {
        match kind {
            // 2回チカチカ → 長休み
            Fatal::ProbeFail => {
                for _ in 0..2 {
                    led_on(led);
                    delay_ms_blocking(80);
                    led_off(led);
                    delay_ms_blocking(120);
                }
                delay_ms_blocking(1_000);
            }
            // 3回チカチカ → 長休み
            Fatal::InitFail => {
                for _ in 0..3 {
                    led_on(led);
                    delay_ms_blocking(80);
                    led_off(led);
                    delay_ms_blocking(120);
                }
                delay_ms_blocking(1_000);
            }
            // 4回チカチカ → 長休み (flushの途中でI2Cが落ちる/ノイズ等)
            Fatal::FlushFail => {
                for _ in 0..4 {
                    led_on(led);
                    delay_ms_blocking(80);
                    led_off(led);
                    delay_ms_blocking(120);
                }
                delay_ms_blocking(1_000);
            }
        }
    }
}

fn blink_nibble(led: &mut Output<'static>, nibble: u8) {
    let n = nibble & 0x0F;
    if n == 0 {
        led_on(led);
        delay_ms_blocking(500);
        led_off(led);
        delay_ms_blocking(300);
        return;
    }
    for _ in 0..n {
        led_on(led);
        delay_ms_blocking(80);
        led_off(led);
        delay_ms_blocking(120);
    }
    delay_ms_blocking(250);
}

fn blink_addr_forever(led: &mut Output<'static>, stage: u8, addr: u8) -> ! {
    // 表示: 上位4bit → 下位4bit を繰り返す
    STAGE.store(stage, Ordering::Relaxed);
    loop {
        blink_nibble(led, addr >> 4);
        blink_nibble(led, addr);
        delay_ms_blocking(900);
    }
}

fn scan_first_i2c_addr(i2c_bus: &'static bus::i2c::I2c1Bus) -> Option<u8> {
    // 一般的な7bitアドレス範囲をスキャン
    let mut dev = CriticalSectionDevice::new(i2c_bus);
    for addr in 0x08u8..=0x77u8 {
        // embassy-rp は 0-length write をエラーにするため 1byte 送る
        // I2Cデバイスに影響が出にくいよう、SSD1306のNOP(0xE3)を使う
        if dev.write(addr, &[0x00, 0xE3]).is_ok() {
            return Some(addr);
        }
    }
    None
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    info!("Program start - Dual Core Test (Core1 LED)");

    // LEDピン
    let led = Output::new(p.PIN_25, Level::Low);

    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C1, p.PIN_7, p.PIN_6, Irqs, i2c_config);

    // Core1起動
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner.spawn(unwrap!(core1_led_task(led)));
                spawner.spawn(unwrap!(core1_oled_task(i2c)));
            });
        },
    );

    // Core0もExecutorを回す（必須）
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(unwrap!(core0_dummy_task()));
    });
}

#[embassy_executor::task]
async fn core0_dummy_task() {
    info!("Core0 task started");
    loop {
        // Core0は死活監視ログを出すだけ
        info!("Core0 is alive");
        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
async fn core1_led_task(mut led: Output<'static>) {
    info!("Core1 LED task started");
    loop {
        led.set_high();
        Timer::after_millis(100).await; // 早めの点滅
        led.set_low();
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn core1_oled_task(i2c: I2c<'static, I2C1, i2c::Async>) {
    use crate::devices::ssd1306::Oled;
    use crate::oled_demo::OledDemo;
    
    info!("Core1 OLED task started");
    match Oled::new(i2c) {
        Ok(mut oled) => {
            let mut demo = OledDemo::new();
            loop {
                match demo.tick(&mut oled) {
                    Ok(delay) => Timer::after_millis(delay).await,
                    Err(_) => {
                        error!("OLED tick failed");
                        Timer::after_millis(1000).await;
                    }
                }
            }
        },
        Err(_) => {
             error!("OLED init failed");
             loop { Timer::after_millis(1000).await; } 
        }
    }
}


/// Program metadata for `picotool info`
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [rp235x_hal::binary_info::EntryAddr; 5] = [
    rp235x_hal::binary_info::rp_cargo_bin_name!(),
    rp235x_hal::binary_info::rp_cargo_version!(),
    rp235x_hal::binary_info::rp_program_description!(c"RP2350 Template"),
    rp235x_hal::binary_info::rp_cargo_homepage_url!(),
    rp235x_hal::binary_info::rp_program_build_attribute!(),
];

// Core0: LED heartbeat (like C++ loop)
#[embassy_executor::task]
async fn heartbeat_task(mut led: Output<'static>) {
    // タスクが実行されたことを確認（すぐにLED点滅開始）
    let mut counter = 0u32;
    loop {
        counter = counter.wrapping_add(1);
        if counter % 2 == 0 {
            led_on(&mut led);
        } else {
            led_off(&mut led);
        }
        embassy_time::Timer::after_millis(200).await; // 200msで高速点滅（動作確認用）
    }
}

// Core1: OLED demo loop (like C++ loop1())
#[embassy_executor::task]
async fn oled_task(i2c_bus: &'static bus::i2c::I2c1Bus, addr: u8) {
    // 診断：oled_task起動 → 80ms点滅
    DELAY_MS.store(80, Ordering::Relaxed);
    
    info!("Core1: oled_task started, addr=0x{:02X}", addr);
    
    // Wait for Core1 to be fully initialized
    embassy_time::Timer::after_millis(100).await;
    
    info!("Core1: Creating I2C device");
    let i2c_dev = CriticalSectionDevice::new(i2c_bus);
    
    info!("Core1: Initializing OLED");
    let mut oled = match devices::ssd1306::Oled::new_at(i2c_dev, addr) {
        Ok(o) => {
            info!("Core1: OLED init OK");
            o
        }
        Err(_) => {
            error!("Core1: OLED init failed");
            // Initialization failed on Core1 - just loop forever
            loop {
                embassy_time::Timer::after_millis(1000).await;
            }
        }
    };

    info!("Core1: Drawing white screen");
    // シンプルな白画面を表示
    oled.clear();
    let style = PrimitiveStyle::with_fill(BinaryColor::On);
    let _ = Rectangle::new(Point::new(0, 0), Size::new(128, 64))
        .into_styled(style)
        .draw(oled.display());
    
    match oled.flush() {
        Ok(_) => info!("Core1: White screen flush OK"),
        Err(_) => error!("Core1: Flush failed"),
    }
    
    // 白画面を保持
    loop {
        embassy_time::Timer::after_millis(1000).await;
    }
}

// End of file
