//  Created by Hasebe Masahiko on 2026/01/03.
//  Copyright (c) 2026 Hasebe Masahiko
//  Released under the MIT license
//  https://opensource.org/licenses/mit-license.php

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

#include "RPi_Pico_TimerInterrupt.h"
#include "configuration.h"
#include "peripheral.h"
#include "sk6812.h"
#include "global_timer.h"
#include "qtouch.h"
#include "constants.h"
#include "midi_if.h"

//--------------------------------------------------------------
//    Variables
//--------------------------------------------------------------
// USB MIDI object
Adafruit_USBD_MIDI usbm;
// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usbm, usbmidi);
#ifdef USE_UART_MIDI
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, uartmidi);
#endif
bool usb_midi_connected = false;

RPI_PICO_Timer ITimer1(1);                    // ハードウェアタイマー #1
volatile uint32_t millisec = 0;                // ISR で変更するので volatile
SK6812 sk(MAX_LIGHT, D0);
uint8_t external_note_status[MAX_MIDI_NOTE] = {0};
GlobalTimer gt;
QubitTouch qt([](uint8_t status, uint8_t note, uint8_t intensity) {
  // MIDI callback function
  send_midi_message(status, note, intensity);
});

// NeoPixel 関連関数のプロトタイプ宣言
void init_neo_pixel();
void clear_touch_leds();
void callback_for_set_led(float locate, int16_t sensor_value);
void set_led_by_accompaniment();
void set_led_by_touch(float locate, int16_t sensor_value, bool touch);
void set_led_for_note(size_t index, uint8_t intensity, bool touch);
void set_white_led(int index, uint8_t intensity);
void set_neo_pixel(int index, int16_t red, int16_t green, int16_t blue, int16_t white);
void update_neo_pixel();
// MIDI ハンドラのプロトタイプ宣言
void handle_note_on(uint8_t channel, uint8_t note, uint8_t velocity);
void handle_note_off(uint8_t channel, uint8_t note, uint8_t velocity);
void handle_program_change(uint8_t channel, uint8_t program);
void handle_note_on_UART(uint8_t channel, uint8_t note, uint8_t velocity);
void handle_note_off_UART(uint8_t channel, uint8_t note, uint8_t velocity);
void handle_control_change_UART(uint8_t channel, uint8_t control, uint8_t value);
void handle_program_change_UART(uint8_t channel, uint8_t program);

//--------------------------------------------------------------
//    Basic Functions
//--------------------------------------------------------------
// --- 割り込みハンドラ ---
#define SPEED 2000       // 2ms ごとに割り込み
bool TimerHandler(struct repeating_timer* /*rt*/) {
  millisec = millisec + 2;
  gt.inc_global_time();
  return true;
}
//--------------------------------------------------------------
void setup() {
  pinMode(LED_HEARTBEAT, OUTPUT);
  gpio_put(LED_HEARTBEAT, HIGH);

  // Initialize USB MIDI
  TinyUSB_Device_Init(0);
  usbm.setStringDescriptor("Loopian::QUBIT2");
  usbmidi.setHandleNoteOn(handle_note_on);
  usbmidi.setHandleNoteOff(handle_note_off);
  usbmidi.setHandleProgramChange(handle_program_change);
  usbmidi.begin(MIDI_CHANNEL_OMNI);
  usbmidi.turnThruOff();

  // wait until device mounted
  for (int i=0; i<1000; i++) {
    if ( TinyUSBDevice.mounted() ) {
      usb_midi_connected = true;
      break;
    }
  }

#ifdef USE_UART_MIDI
  uartmidi.setHandleNoteOn(handle_note_on_UART);
  uartmidi.setHandleNoteOff(handle_note_off_UART);
  uartmidi.setHandleControlChange(handle_control_change_UART);
  uartmidi.setHandleProgramChange(handle_program_change_UART);
  uartmidi.begin(MIDI_CHANNEL_OMNI);
  uartmidi.turnThruOff();
#endif

  init_neo_pixel();
}
//--------------------------------------------------------------
void loop() {
  gt.update_timer();

  if (usb_midi_connected) {
    // MIDI メッセージの処理
    usbmidi.read();
  }
#ifdef USE_UART_MIDI
  uartmidi.read();
#endif

  // Heartbeat LED
  if (gt.timer_100msec_event()) {
    if (gt.timer_100msec()%2 == 0) {
      gpio_put(LED_HEARTBEAT, LOW);
      send_note_on(1, 60, 100);
    }
    else {
      gpio_put(LED_HEARTBEAT, HIGH);
      send_note_off(1, 60);
    }
  }
}

//--------------------------------------------------------------
//    Basic Functions(Core 1) for I2C
//--------------------------------------------------------------
void setup1() {
  wireBegin();

  // Wait for display
  delay(500);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // 2ms ごとに TimerHandler を呼び出し
  if (!ITimer1.attachInterruptInterval(SPEED, TimerHandler)) {
    // 失敗したら止める
    while (true) {}
  }
}
void loop1() {
  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);
  // display.display() is NOT necessary after every single drawing command,
  // unless that's what you want...rather, you can batch up a bunch of
  // drawing operations and then update the screen all at once by calling
  // display.display(). These examples demonstrate both approaches...

  testdrawline();      // Draw many lines

  testdrawrect();      // Draw rectangles (outlines)

  testfillrect();      // Draw rectangles (filled)

  testdrawcircle();    // Draw circles (outlines)

  testfillcircle();    // Draw circles (filled)

  testdrawroundrect(); // Draw rounded rectangles (outlines)

  testfillroundrect(); // Draw rounded rectangles (filled)

  testdrawtriangle();  // Draw triangles (outlines)

  testfilltriangle();  // Draw triangles (filled)

  testdrawchar();      // Draw characters of the default font

  testdrawstyles();    // Draw 'stylized' characters

  testscrolltext();    // Draw scrolling text

  testdrawbitmap();    // Draw a small bitmap image

  // Invert and restore display, pausing in-between
  display.invertDisplay(true);
  delay(1000);
  display.invertDisplay(false);
  delay(1000);

  testanimate(); // Animate bitmaps
}

/*----------------------------------------------------------------------------*/
//     NeoPixel
/*----------------------------------------------------------------------------*/
uint8_t neo_pixel[MAX_LIGHT][4];
enum LED_STATUS { NO_STATUS, TOUCH_STATUS, ACCOMPANIMENT_STATUS };
LED_STATUS led_status[MAX_LIGHT] = { NO_STATUS }; // Status of each sensor for LED control
void init_neo_pixel() {
  // Set up the sk6812
  sk.begin();
  for (int i = 0; i < MAX_LIGHT; i++) {
    neo_pixel[i][0] = 0; // red
    neo_pixel[i][1] = 0; // green
    neo_pixel[i][2] = 0; // blue
    neo_pixel[i][3] = 0; // white
  }
  update_neo_pixel();
}
void clear_touch_leds() {
    for (int i = 0; i < MAX_LIGHT; i++) {
      neo_pixel[i][0] = 0; // red
      neo_pixel[i][1] = 0; // green
      neo_pixel[i][2] = 0; // blue
      led_status[i] = NO_STATUS;
  }
}
//-----------------------------------------------------------
void callback_for_set_led(float locate, int16_t sensor_value) {
  set_led_by_touch(locate, sensor_value, true);
}
void set_led_by_accompaniment() {
  for (int i = 0; i < MAX_SENS; i++) {
    int idx = i + KEYBD_LO - 4;
    if (external_note_status[idx] > 0) {
      set_led_by_touch(i, external_note_status[idx], false);
    }
  }
}
//-----------------------------------------------------------
void set_led_by_touch(float locate, int16_t sensor_value, bool touch) {
  if ((locate < 0.0f) || (locate >= static_cast<float>(MAX_SENS))){
    return; // Invalid location
  }
  if (sensor_value <= 1) {
    sensor_value = 1;
  }

  const float SLOPE = 20000.0f / sensor_value; // 傾き:小さいほどたくさん光る
  float nearest_lower = std::floor(locate);
  float nearest_upper = std::ceil(locate);

  while (1) {
    int16_t this_val = static_cast<int16_t>(255 - (locate - nearest_lower)*SLOPE);
    if (this_val < 0) {break;}
    int index = static_cast<int>(nearest_lower);
    nearest_lower -= 1.0f;
    if (led_status[index%MAX_LIGHT] == TOUCH_STATUS && !touch) {
      // すでにタッチセンサで点灯している場合、伴奏は表示しない
      continue;
    }
    set_led_for_note(index, this_val, touch);
    led_status[index%MAX_LIGHT] = touch ? TOUCH_STATUS : ACCOMPANIMENT_STATUS;
  }
  while (1) {
    int16_t this_val = static_cast<int16_t>(255 - (nearest_upper - locate)*SLOPE);
    if (this_val < 0) {break;}
    int index = static_cast<int>(nearest_upper);
    nearest_upper += 1.0f;
    if (led_status[index%MAX_LIGHT] == TOUCH_STATUS && !touch) {
      // すでにタッチセンサで点灯している場合、伴奏は表示しない
      continue;
    }
    set_led_for_note(index, this_val, touch);
    led_status[index%MAX_LIGHT] = touch ? TOUCH_STATUS : ACCOMPANIMENT_STATUS;
  }
}
//-----------------------------------------------------------
void set_led_for_note(size_t index, uint8_t intensity, bool touch) {
  uint8_t red = 0;
  uint8_t blue = 0;
  uint8_t green = 0;

  if (touch) {
    // QUBIT Touch : Magenta
    red = (intensity * 4) / 5;
    blue = (intensity * 1) / 5;
  } else {
    // QUBIT Accompaniment : Cyan
    blue = (intensity * 3) / 5;
    green = (intensity * 2) / 5;
  }
  set_neo_pixel(index, red, green, blue, -1); // Set only red and blue channels
}
//-----------------------------------------------------------
void set_led_for_wave(uint16_t global_time) {
  float tm = static_cast<float>(global_time); // convert to seconds
  for (int i = 0; i < MAX_LIGHT; i++) {
    float phase = (tm * 0.002f + static_cast<float>(i) * 0.1f) * 2 * PI;
    uint8_t intensity = static_cast<uint8_t>(10.0f * (std::sin(phase)) + 10.0f);
    set_white_led(i, intensity);
  }
}
//-----------------------------------------------------------
void set_white_led(int index, uint8_t intensity) {
  set_neo_pixel(index, -1, -1, -1, intensity); // Set only red and blue channels
}
//-----------------------------------------------------------
void set_neo_pixel(int index, int16_t red, int16_t green, int16_t blue, int16_t white) {
  while (index < 0) {
    index += MAX_LIGHT; // Wrap around if negative
  }
  index %= MAX_LIGHT;
  if (red != -1)    {neo_pixel[index][0] = static_cast<uint8_t>(red);}
  if (green != -1)  {neo_pixel[index][1] = static_cast<uint8_t>(green);}
  if (blue != -1)   {neo_pixel[index][2] = static_cast<uint8_t>(blue);}
  if (white != -1)  {neo_pixel[index][3] = static_cast<uint8_t>(white);}
}
void update_neo_pixel() {
  sk.clear();
  for (int i = 0; i < MAX_LIGHT; i++) {
    sk.setPixelColor(i, neo_pixel[i][0], neo_pixel[i][1], neo_pixel[i][2], neo_pixel[i][3]);
  }
  sk.show();
}

/*----------------------------------------------------------------------------*/
//     MIDI Handlers
/*----------------------------------------------------------------------------*/
void handle_note_on(uint8_t channel, uint8_t note, uint8_t velocity) {}
void handle_note_off(uint8_t channel, uint8_t note, uint8_t velocity) {}
void handle_program_change(uint8_t channel, uint8_t program) {}
void handle_note_on_UART(uint8_t channel, uint8_t note, uint8_t velocity) {}
void handle_note_off_UART(uint8_t channel, uint8_t note, uint8_t velocity) {}
void handle_control_change_UART(uint8_t channel, uint8_t control, uint8_t value) {}
void handle_program_change_UART(uint8_t channel, uint8_t program) {}
/*----------------------------------------------------------------------------*/
void send_note_on(uint8_t channel, uint8_t note, uint8_t vel) {
  usbmidi.sendNoteOn(note, vel, channel);
}
void send_note_off(uint8_t channel, uint8_t note) {
  usbmidi.sendNoteOff(note, 64, channel);
}
void send_control_change(uint8_t channel, uint8_t controller, uint8_t value) {
  usbmidi.sendControlChange(controller, value, channel);
}
void send_program_change(uint8_t channel, uint8_t pcn) {
  usbmidi.sendProgramChange(pcn, channel);
}
void send_note_on_UART(uint8_t channel, uint8_t pitch, uint8_t velocity) {
#ifdef USE_UART_MIDI
  uartmidi.sendNoteOn(pitch, velocity, channel);
#endif
}
void send_note_off_UART(uint8_t channel, uint8_t pitch, uint8_t velocity) {
#ifdef USE_UART_MIDI
  uartmidi.sendNoteOff(pitch, velocity, channel);
#endif
}
void send_control_change_UART(uint8_t channel , uint8_t number , uint8_t value ) {
#ifdef USE_UART_MIDI
  uartmidi.sendControlChange(number, value, channel);
#endif
}
void send_program_change_UART(uint8_t channel , uint8_t number) {
#ifdef USE_UART_MIDI
  uartmidi.sendProgramChange(number, channel);
#endif
}
void send_midi_message(uint8_t status, uint8_t note, uint8_t velocity) {
  uint8_t channel = (status & 0x0F) + 1;
  switch(status & 0xF0) {
      case 0x90: // Note On
          //send_note_on_UART(channel, note, velocity);
          break;
      case 0x80: // Note Off
          //send_note_off_UART(channel, note, velocity);
          break;
      case 0xB0: // Control Change
          //send_control_change_UART(channel, note, velocity);
          break;
      case 0xC0: // Program Change
          //send_program_change_UART(channel, note);
          break;
      default:
          break;
  }
}
