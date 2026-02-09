//  Created by Hasebe Masahiko on 2026/01/03.
//  Copyright (c) 2026 Hasebe Masahiko
//  Released under the MIT license
//  https://opensource.org/licenses/mit-license.php

#include  "Arduino.h"
#include  <Wire.h>
#include  "peripheral.h"
#include  "configuration.h"

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//  RP2040 を使う時は Wire. で、RP2350 を使う時は Wire1. にする
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//---------------------------------------------------------
//		Initialize I2C Device
//---------------------------------------------------------
void wireBegin( void )
{
  Wire1.setClock(I2C_CLOCK_HZ);
  Wire1.setSDA(I2C_SDA_PIN);
  Wire1.setSCL(I2C_SCL_PIN);
  Wire1.begin();
}
static void scanI2C(TwoWire &wire, Stream &out)
{
  out.println("I2C scan start");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    wire.beginTransmission(addr);
    const uint8_t err = wire.endTransmission();
    if (err == 0) {
      out.print("  found: 0x");
      if (addr < 16) out.print('0');
      out.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    out.println("  (no devices)");
  }
  out.println("I2C scan end");
}

//---------------------------------------------------------
//		Write I2C Device
//    Err Code
//      0:success
//      1:data too long to fit in transmit buffer
//      2:received NACK on transmit of address
//      3:received NACK on transmit of data
//      4:other error
//---------------------------------------------------------
int write_i2cDevice( unsigned char adrs, unsigned char* buf, int count )
{
	Wire1.beginTransmission(adrs);
  Wire1.write(buf,count);
	return Wire1.endTransmission();
}
//---------------------------------------------------------
//		Read 1byte I2C Device
//---------------------------------------------------------
int read1byte_i2cDevice( unsigned char adrs, unsigned char* wrBuf, unsigned char* rdBuf, int wrCount )
{
	unsigned char err;

	Wire1.beginTransmission(adrs);
  Wire1.write(wrBuf,wrCount);
	err = Wire1.endTransmission(false);
	if ( err != 0 ){ return err; }

	err = Wire1.requestFrom(adrs,(uint8_t)1,(uint8_t)0);
	while(Wire1.available()) {
		*rdBuf = Wire1.read();
	}

	//err = Wire1.endTransmission(true);
	//return err;
  return 0;
}
//---------------------------------------------------------
//		Read N byte I2C Device
//---------------------------------------------------------
//送信結果 (byte) 
//0: 成功 
//1: 送ろうとしたデータが送信バッファのサイズを超えた 
//2: スレーブ・アドレスを送信し、NACKを受信した 
//3: データ・バイトを送信し、NACKを受信した 
//4: その他のエラー 
//
int read_nbyte_i2cDevice( unsigned char adrs, unsigned char* wrBuf, unsigned char* rdBuf, int wrCount, int rdCount )
{
	unsigned char err;

	Wire1.beginTransmission(adrs);
  Wire1.write(wrBuf,wrCount);
	err = Wire1.endTransmission(false);
	if ( err != 0 ){ return err; }

	err = Wire1.requestFrom(adrs,static_cast<uint8_t>(rdCount),(uint8_t)0);
	int rdAv = 0;
	while((rdAv = Wire1.available()) != 0) {
		*(rdBuf+rdCount-rdAv) = Wire1.read();
	}

	//err = Wire1.endTransmission(true);
	//return err;

	return 0;
}
// ハングしてはいけない本番用
int read_nbyte_i2cDeviceX( unsigned char adrs, unsigned char* wrBuf, unsigned char* rdBuf, int wrCount, int rdCount )
{
	unsigned char err;
  volatile int cnt=0;

	Wire1.beginTransmission(adrs);
  Wire1.write(wrBuf,wrCount);
	err = Wire1.endTransmission(false);
	if ( err != 0 ){ return err; }

	err = Wire1.requestFrom(adrs,static_cast<uint8_t>(rdCount),(uint8_t)0);
	int rdAv = 0;
	while(((rdAv = Wire1.available()) != 0) && (cnt < 10)){
		*(rdBuf+rdCount-rdAv) = Wire1.read();
    cnt += 1;
	}

	return 0;
}
//---------------------------------------------------------
//    Read Only N byte I2C Device
//    Err Code
//      0:success
//      1:data too long to fit in transmit buffer
//      2:received NACK on transmit of address
//      3:received NACK on transmit of data
//      4:other error
//---------------------------------------------------------
int read_only_nbyte_i2cDevice( unsigned char adrs, unsigned char* rdBuf, int rdCount )
{
  unsigned char err;

  err = Wire1.requestFrom(adrs,static_cast<uint8_t>(rdCount),static_cast<uint8_t>(false));
  int rdAv = Wire1.available();
  while( rdAv ) {
    *(rdBuf+rdCount-rdAv) = Wire1.read();
    rdAv--;
  }

  err = Wire1.endTransmission(true);
  return err;
}

//-------------------------------------------------------------------------
//			AT42QT1070
//-------------------------------------------------------------------------
#ifdef USE_AT42QT1070
static const uint8_t AT42QT_I2C_ADRS = 0x1B;
// read
static const uint8_t AT42QT_STATUS = 2;

static const uint8_t AT42QT_LP_MODE = 54;
static const uint8_t AT42QT_MAX_DUR = 55;

void AT42QT_init( void )
{
  uint8_t i2cdata[2] = {AT42QT_LP_MODE, 0};
  int err = write_i2cDevice(AT42QT_I2C_ADRS, i2cdata,2);
  if ( err != 0 ){ return; }

  i2cdata[0] = AT42QT_MAX_DUR;
  err = write_i2cDevice(AT42QT_I2C_ADRS, i2cdata, 2);
  if ( err != 0 ){ return; }
}

int AT42QT_read( size_t key, uint8_t (&rdraw)[2], bool ref )
{
  uint8_t wd = 0;
  if (ref) {
    wd = 18 + key*2;
  } else {
    wd = 4 + key*2;  //  0-6
  }  
  return read_nbyte_i2cDeviceX(AT42QT_I2C_ADRS, &wd, rdraw, 1, 2);
}
#endif

//-------------------------------------------------------------------------
//			SSD1306
//-------------------------------------------------------------------------
#ifdef USE_SSD1306
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire1, OLED_RESET);
#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

void testdrawline() {
  int16_t i;

  display.clearDisplay(); // Clear display buffer

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn line
    delay(1);
  }
  for(i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000); // Pause for 2 seconds
}

void testdrawrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testfillrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=3) {
    // The INVERSE color is used so rectangles alternate white/black
    display.fillRect(i, i, display.width()-i*2, display.height()-i*2, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testdrawcircle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=2) {
    display.drawCircle(display.width()/2, display.height()/2, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillcircle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=3) {
    // The INVERSE color is used so circles alternate white/black
    display.fillCircle(display.width() / 2, display.height() / 2, i, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn circle
    delay(1);
  }

  delay(2000);
}

void testdrawroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    display.drawRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    // The INVERSE color is used so round-rects alternate white/black
    display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawtriangle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=5) {
    display.drawTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfilltriangle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=5) {
    // The INVERSE color is used so triangles alternate white/black
    display.fillTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawchar(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
  delay(2000);
}

void testdrawstyles(void) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Hello, world!"));

  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.println(3.141592);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.print(F("0x")); display.println(0xDEADBEEF, HEX);

  display.display();
  delay(2000);
}

void testscrolltext(void) {
  display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println(F("scroll"));
  display.display();      // Show initial text
  delay(100);

  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrolldiagright(0x00, 0x07);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x07);
  delay(2000);
  display.stopscroll();
  delay(1000);
}

void testdrawbitmap(void) {
  display.clearDisplay();

  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(1000);
}

#define XPOS   0 // Indexes into the 'icons' array in function below
#define YPOS   1
#define DELTAY 2

void testanimate(void) {
    const uint8_t *bitmap = logo_bmp;
    uint8_t w = LOGO_WIDTH;
    uint8_t h = LOGO_HEIGHT;

    int8_t f, icons[NUMFLAKES][3];

  // Initialize 'snowflake' positions
  for(f=0; f< NUMFLAKES; f++) {
    icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
    icons[f][YPOS]   = -LOGO_HEIGHT;
    icons[f][DELTAY] = random(1, 6);
    Serial.print(F("x: "));
    Serial.print(icons[f][XPOS], DEC);
    Serial.print(F(" y: "));
    Serial.print(icons[f][YPOS], DEC);
    Serial.print(F(" dy: "));
    Serial.println(icons[f][DELTAY], DEC);
  }

  for(int c=0; c<100; c++) {
    display.clearDisplay(); // Clear the display buffer

    // Draw each snowflake:
    for(f=0; f< NUMFLAKES; f++) {
      display.drawBitmap(icons[f][XPOS], icons[f][YPOS], bitmap, w, h, SSD1306_WHITE);
    }

    display.display(); // Show the display buffer on the screen
    delay(200);        // Pause for 1/10 second

    // Then update coordinates of each flake...
    for(f=0; f< NUMFLAKES; f++) {
      icons[f][YPOS] += icons[f][DELTAY];
      // If snowflake is off the bottom of the screen...
      if (icons[f][YPOS] >= display.height()) {
        // Reinitialize to a random position, just off the top
        icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
        icons[f][YPOS]   = -LOGO_HEIGHT;
        icons[f][DELTAY] = random(1, 6);
      }
    }
  }
}
#endif