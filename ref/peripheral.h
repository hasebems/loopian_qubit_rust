#pragma once
#include <Adafruit_SSD1306.h>

// I2C (RP2350: use Wire1 on Seeed XIAO RP2350)
#define I2C_CLOCK_HZ 400000
#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7

void wireBegin( void );

// SSD1306 OLED size
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
extern Adafruit_SSD1306 display;
void testdrawline(void);
void testdrawrect(void);
void testfillrect(void);
void testdrawcircle(void);
void testfillcircle(void);
void testdrawroundrect(void);
void testfillroundrect(void);
void testdrawtriangle(void);
void testfilltriangle(void);
void testdrawchar(void);
void testdrawstyles(void);
void testscrolltext(void);
void testdrawbitmap(void);
void testanimate(void);
