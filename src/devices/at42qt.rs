/// AT42QT1070 （ただし実体はMUXの向こうにあるので addr は固定で良い）
pub struct At42Qt1070 {
    addr: u8, // 固定I2Cアドレス
}

impl At42Qt1070 {
    const ADDR: u8 = 0x1B;
    pub const fn new() -> Self { Self { addr: Self::ADDR } }

    /// 初期化
    pub async fn init<I2C>(&mut self, _i2c: &mut I2C) -> Result<(), I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        // i2c.write(self.addr, &[reg, val]).await?;
        // i2c.write(self.addr, &[...]).await?;
        Ok(())
    }

    /// 7キー分などの「現在状態ビット」を返す（例：bit0..bit6）
    pub async fn read_state<I2C>(&mut self, i2c: &mut I2C) -> Result<u8, I2C::Error>
    where
        I2C: embedded_hal_async::i2c::I2c,
    {
        let mut buf = [0u8; 1];
        i2c.write_read(self.addr, &[0], &mut buf).await?;
        Ok(buf[0])
    }
}