pub struct Pca9544 {
    addr: u8,
}

impl Pca9544 {
    const ADDR: u8 = 0x74;
    pub const fn new() -> Self {
        Self { addr: Self::ADDR }
    }

    pub async fn select<I2C>(&self, i2c: &mut I2C, ch: u8) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        let ctrl = self.addr + ch;
        i2c.write(self.addr, &[ctrl]).await
    }
}
