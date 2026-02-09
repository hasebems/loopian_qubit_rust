use core::cell::RefCell;

use critical_section::Mutex;
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::I2C1;
use embassy_sync::mutex::Mutex as AsyncMutex;
use static_cell::StaticCell;

pub type I2c1Blocking = I2c<'static, I2C1, i2c::Blocking>;
pub type I2c1Bus = Mutex<RefCell<I2c1Blocking>>;

pub type I2c1Async = I2c<'static, I2C1, i2c::Async>;
pub type I2c1BusAsync = AsyncMutex<embassy_sync::blocking_mutex::raw::NoopRawMutex, I2c1Async>;

static I2C1_BUS: StaticCell<I2c1Bus> = StaticCell::new();
static I2C1_BUS_ASYNC: StaticCell<I2c1BusAsync> = StaticCell::new();

pub fn init_i2c1_bus(i2c: I2c1Blocking) -> &'static I2c1Bus {
    I2C1_BUS.init(Mutex::new(RefCell::new(i2c)))
}

pub fn init_i2c1_bus_async(i2c: I2c1Async) -> &'static I2c1BusAsync {
    I2C1_BUS_ASYNC.init(AsyncMutex::new(i2c))
}
