use display_interface_i2c::I2CInterface;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::*;
use ssd1306::{I2CDisplayInterface, Ssd1306};

pub type OledDisplay<I2C> =
    Ssd1306<I2CInterface<I2C>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;

pub struct Oled<I2C> {
    display: OledDisplay<I2C>,
}

impl<I2C> Oled<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    pub fn new(i2c: I2C) -> Result<Self, display_interface::DisplayError> {
        Self::new_at(i2c, 0x3C)
    }

    pub fn new_at(i2c: I2C, addr: u8) -> Result<Self, display_interface::DisplayError> {
        let interface = match addr {
            0x3D => I2CDisplayInterface::new_alternate_address(i2c),
            _ => I2CDisplayInterface::new(i2c),
        };
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display.init()?;
        // まずは視認性優先で最大輝度に寄せる（配線確認・初期動作確認用）
        let _ = display.set_brightness(Brightness::BRIGHTEST);
        let _ = display.set_display_on(true);
        display.clear_buffer();
        display.flush()?;

        Ok(Self { display })
    }

    pub fn display(&mut self) -> &mut OledDisplay<I2C> {
        &mut self.display
    }

    pub fn clear(&mut self) {
        self.display.clear_buffer();
    }

    pub fn flush(&mut self) -> Result<(), display_interface::DisplayError> {
        self.display.flush()
    }
}
