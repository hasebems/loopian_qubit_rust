# Rust pico_template

## 概要

- Seeed XIAO RP2350 を組み込みRustで動作させます
- 擬似マルチタスク Embassy を Multicore 上で動作させるためのテンプレートです
- 主に GPIO, I2C, USB MIDI 等の下回りにアクセスする処理を提供します

## 提供機能（タスク）詳細

### I2C

- SSD1306 による OLED Display の表示機能
- AT42QT1070 によるタッチセンサー機能
    - PCA9544 により複数個のセンサーを読み込み可能

### UI

- SSD1306 に提供する画像を生成

