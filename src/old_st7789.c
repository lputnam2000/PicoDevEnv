/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdarg.h>
#include <math.h>
// you need to change cmakelists.txt
// to add new headers
#include "pico.h"
#include "pico/stdlib.h"
#include "pico/multicore.h" // pico_multicore
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/interp.h"
#include "hardware/spi.h" // hardware_spi
#include "hardware/adc.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/flash.h"
#include "hardware/vreg.h"
#include "st7789_lcd.pio.h"
#include "raspberry_256x256_rgb565.h"
//#include "rpi_256_256_Palette.h"
#define USE_PIO
#define FLASH_TARGET_OFFSET (256 * 1024)
const uint8_t *FlashRam = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);

void WriteFlash()
{
  flash_range_program(FLASH_TARGET_OFFSET, raspberry_256x256, FLASH_PAGE_SIZE*256*2);
}
uint16_t _hw = 240;
uint16_t _hh = 320;
uint16_t _width = 240;
uint16_t _height = 240;

typedef struct
{
  // Pins
  int CS;
  int DS;
  // IO
  int SPINO;   // 0,1 = spi0,spi1 2=PIO
  int SPIMODE; // 0=1 1=Rev Pol
  // disp info
  int Width;
  int Height;
} st7789_lcd;

st7789_lcd Displays[4];
st7789_lcd *CurDisplay = Displays;
int DisplayCount = 0;

// Init a new st7789 display
int Init7789(int CLK, int MOSI, int RES, int DS, int CS, int SPINO, int SPIMODE, int W, int H)
{
  if (DisplayCount > 3)
    return -1;
  st7789_lcd *CurDisplay = &Displays[DisplayCount++];
  return DisplayCount - 1;
}

// Set which display the drawing functions should write to
int Use7789(int DispNo)
{
}

void Blit7789()
{
}

#define PROGMEM
typedef struct
{
  uint16_t bitmapOffset; ///< Pointer into GFXfont->bitmap
  uint8_t width;         ///< Bitmap dimensions in pixels
  uint8_t height;        ///< Bitmap dimensions in pixels
  uint8_t xAdvance;      ///< Distance to advance cursor (x axis)
  int8_t xOffset;        ///< X dist from cursor pos to UL corner
  int8_t yOffset;        ///< Y dist from cursor pos to UL corner
} GFXglyph;

/// Data stored for FONT AS A WHOLE
typedef struct
{
  uint8_t *bitmap;  ///< Glyph bitmaps, concatenated
  GFXglyph *glyph;  ///< Glyph array
  uint16_t first;   ///< ASCII extents (first char)
  uint16_t last;    ///< ASCII extents (last char)
  uint8_t yAdvance; ///< Newline distance (y axis)
} GFXfont;
#include "Fonts/FreeSerifBold12pt7b.h"
GFXfont *gfxFont = (GFXfont *)&FreeSerifBold12pt7b;
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240
#define IMAGE_SIZE 256
#define LOG_IMAGE_SIZE 8

// SCL d18
// SDA d23
// RES d4
// DC  d2
const uint LED_PIN = 25;

#define PIN_BL 1

#define SPI_PORT spi0
#define PIN_CLK 18
#define PIN_DIN 19
#define PIN_RESET 20
#define PIN_CS 21
#define PIN_DC 27
/*
#define SPI_PORT spi1
#define PIN_CS 0
#define PIN_CLK 10
#define PIN_DIN 11
#define PIN_RESET 12
#define PIN_DC 13
*/

#define SERIAL_CLK_DIV 1.f
#define READ_BIT 0x80
// Format: cmd length (including cmd byte), post delay in units of 5 ms, then cmd payload
// Note the delays have been shortened a little

// SDS format is = length, sleep time * 5 ms, command list....

#define eswap(val) ((val >> 8) | (val << 8))
//#define eswap(val) (val)

const uint8_t MADCTL_MY = (0x80);
const uint8_t MADCTL_MX = (0x40);
const uint8_t MADCTL_MV = (0x20);
const uint8_t MADCTL_ML = (0x10);

static const uint8_t st7789_init_seq[] =
    {
        1, 30, 0x01,                       // Software reset 150ms
        1, 2, 0x11,                        // Exit sleep mode
        2, 2, 0x3A, 0x55,                  // Set colour mode to (65K) | (16 bit)
        2, 2, 0x36, MADCTL_MY | MADCTL_MX, // Set MADCTL: row then column, refresh is bottom to top ????
        5, 2, 0x2a, 0x00, 00, 0, 240,      // CASET: column addresses from 0 to 240 (f0)
        5, 2, 0x2b, 0x00, 00, 0x1, 0x40,   // RASET: row addresses from 0 to 240 (f0)
        1, 2, 0x21,                        // Inversion on, then 10 ms delay (supposedly a hack?)
        1, 2, 0x13,                        // Normal display on, then 10 ms delay
        1, 2, 0x29,                        // Main screen turn on, then wait 500 ms
        //1, 100, 0x28,                           // Main screen turn off, then wait 500 ms
        //1, 100, 0x29,                           // Main screen turn on, then wait 500 ms
        //1, 100, 0x28,                           // Main screen turn off, then wait 500 ms
        //1, 100, 0x29,                           // Main screen turn on, then wait 500 ms
        //1, 100, 0x28,                           // Main screen turn off, then wait 500 ms
        //1, 100, 0x29,                           // Main screen turn on, then wait 500 ms
        //1, 100, 0x28,                           // Main screen turn off, then wait 500 ms
        //1, 100, 0x29,                           // Main screen turn on, then wait 500 ms
        0 // Terminate list
};

void HardReset()
{
  gpio_put(PIN_CS, 0);
  gpio_put(PIN_RESET, 1);
  sleep_ms(50);
  gpio_put(PIN_RESET, 0);
  sleep_ms(50);
  gpio_put(PIN_RESET, 1);
  sleep_ms(150);
  gpio_put(PIN_CS, 1);
}
static inline void lcd_set_dc_cs(bool dc, bool cs)
{
  sleep_us(1);
  gpio_put_masked((1u << PIN_DC) | (1u << PIN_CS), !!dc << PIN_DC | !!cs << PIN_CS);
  sleep_us(1);
}

static inline void lcd_write_cmd(PIO pio, uint sm, const uint8_t *cmd, size_t count)
{
  //#if USE_PIO
  //st7789_lcd_wait_idle(pio, sm);
  //st7789_lcd_put(pio, sm, *cmd++);
  //#else
  lcd_set_dc_cs(0, 0);
  spi_write_blocking(SPI_PORT, cmd, 1);

  //#endif
  if (count >= 2)
  {
    //   st7789_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(1, 0);
    //  for (size_t i = 0; i < count - 1; ++i)
    //st7789_lcd_put(pio, sm, *cmd++);
    spi_write_blocking(SPI_PORT, cmd + 1, count - 1);
  }
  // st7789_lcd_wait_idle(pio, sm);
  lcd_set_dc_cs(1, 1);
}

static inline void lcd_init(PIO pio, uint sm, const uint8_t *init_seq)
{
  const uint8_t *cmd = init_seq;
  while (*cmd)
  {
    lcd_write_cmd(pio, sm, cmd + 2, *cmd);
    sleep_ms(*(cmd + 1) * 5);
    cmd += *cmd + 2;
  }
}
void WriteCmd8(uint8_t cmd, uint8_t data)
{
  lcd_set_dc_cs(0, 0);
  spi_write_blocking(SPI_PORT, &cmd, 1);
  lcd_set_dc_cs(1, 0);
  spi_write_blocking(SPI_PORT, (uint8_t *)&data, 1);
  lcd_set_dc_cs(1, 1);
}
void WriteCmd(uint8_t cmd, uint32_t data)
{
  lcd_set_dc_cs(0, 0);
  spi_write_blocking(SPI_PORT, &cmd, 1);
  lcd_set_dc_cs(1, 0);
  spi_write_blocking(SPI_PORT, (uint8_t *)&data, 4);
  lcd_set_dc_cs(1, 1);
}

void SetOrientation(uint8_t ori)
{
  ori &= 3;
  if (ori == 0)
  {
    _width = 240;
    _height = 240;
    WriteCmd8(0x36, 0);
  }
  if (ori == 1)
  {
    _width = 320;
    _height = 180;
    WriteCmd8(0x36, MADCTL_MX | MADCTL_MV);
  }
  if (ori == 2)
  {
    _width = 240;
    _height = 240;
    WriteCmd8(0x36, MADCTL_MX | MADCTL_MY);
  }
  if (ori == 3)
  {
    _width = 320;
    _height = 180;
    WriteCmd8(0x36, MADCTL_MY | MADCTL_MV);
  }
}
void SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  uint32_t xstart = 0;
  uint32_t ystart = 0;
  //x = 0;//xstart;
  //y = 0;//_ystart;
  //uint32_t xa = ((uint32_t)x << 16) | (x + w - 1);
  //uint32_t ya = ((uint32_t)y << 16) | (y + h - 1);

  uint8_t bufx[4] = {(x0 + xstart) >> 8, (x0 + xstart) & 0xFF, (x1 + xstart) >> 8, (x1 + xstart) & 0xFF};
  uint8_t bufy[4] = {(y0 + ystart) >> 8, (y0 + ystart) & 0xFF, (y1 + ystart) >> 8, (y1 + ystart) & 0xFF};
  uint8_t cmd = 0x2A;

  //WriteCmd(0x2A,xa);
  //WriteCmd(0x2B,ya);
  lcd_set_dc_cs(0, 0);
  spi_write_blocking(SPI_PORT, &cmd, 1);
  lcd_set_dc_cs(1, 0);
  spi_write_blocking(SPI_PORT, bufx, 4);
  cmd = 0x2B;
  lcd_set_dc_cs(0, 0);
  spi_write_blocking(SPI_PORT, &cmd, 1);
  lcd_set_dc_cs(1, 0);
  spi_write_blocking(SPI_PORT, bufy, 4);

  cmd = 0x2C;
  lcd_set_dc_cs(0, 0);
  spi_write_blocking(SPI_PORT, &cmd, 1);
  lcd_set_dc_cs(1, 0);
}
void StartPixels()
{
  uint8_t cmd = 0x2C;
  lcd_set_dc_cs(0, 0);
  spi_write_blocking(SPI_PORT, &cmd, 1);
  lcd_set_dc_cs(1, 0);
}
static inline void st7789_start_pixels(PIO pio, uint sm)
{
  uint8_t cmd = 0x2c; // RAMWR
  lcd_write_cmd(pio, sm, &cmd, 1);
  lcd_set_dc_cs(1, 0);
}
//
// GFX
//
const uint16_t cBlack = 0x0000;
const uint16_t cWhite = 0xffff;
const uint16_t cRed = 0xF800;
const uint16_t cGreen = 0x07E0;
const uint16_t cBlue = 0x001F;
const uint16_t cYellow = 0xFFE0;
const uint16_t cCyan = 0x07FF;
const uint16_t cMagenta = 0xF81F;
const uint16_t cOrange = 0xFD20;
const uint16_t cLightGrey = 0xC618;
const uint16_t cDarkGrey = 0x7BEF;
// Rasp Image = 128kb +115kb = 243kb
// Rasp Image = 128kb +153kb = 243kb

uint16_t buf[240 * 240]; //153kb

void PutPixel16(uint16_t x, uint16_t y, uint16_t col)
{
  if (x < _width && y < _height && x >= 0 && y >= 0)
    buf[x + (y * _width)] = eswap(col);
}

#define pgm_read_byte *(uint8_t *)
#define pgm_read_word *(uint16_t *)
inline GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c)
{

  // expression in __AVR__ section may generate "dereferencing type-punned
  // pointer will break strict-aliasing rules" warning In fact, on other
  // platforms (such as STM32) there is no need to do this pointer magic as
  // program memory may be read in a usual way So expression may be simplified
  return gfxFont->glyph + c;
}

inline uint8_t *pgm_read_bitmap_ptr(const GFXfont *gfxFont)
{
  return gfxFont->bitmap;
}

void setFont(const GFXfont *f)
{
  gfxFont = (GFXfont *)f;
}

void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y)
{

  c -= (uint8_t)pgm_read_byte(&gfxFont->first);
  GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c);
  uint8_t *bitmap = pgm_read_bitmap_ptr(gfxFont);

  uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
  uint8_t w = pgm_read_byte(&glyph->width), h = pgm_read_byte(&glyph->height);
  int8_t xo = pgm_read_byte(&glyph->xOffset),
         yo = pgm_read_byte(&glyph->yOffset);
  uint8_t xx, yy, bits = 0, bit = 0;
  int16_t xo16 = 0, yo16 = 0;

  if (size_x > 1 || size_y > 1)
  {
    xo16 = xo;
    yo16 = yo;
  }

  for (yy = 0; yy < h; yy++)
  {
    for (xx = 0; xx < w; xx++)
    {
      if (!(bit++ & 7))
      {
        bits = pgm_read_byte(&bitmap[bo++]);
      }
      if (bits & 0x80)
      {
        if (size_x == 1 && size_y == 1)
        {
          PutPixel16(x + xo + xx, y + yo + yy, color);
        }
        else
        {
          //writeFillRect(x + (xo16 + xx) * size_x, y + (yo16 + yy) * size_y, size_x, size_y, color);
        }
      }
      bits <<= 1;
    }
  }
}
uint16_t cursor_x = 10;
uint16_t cursor_y = 10;
uint16_t textcolor = cBlack;
uint16_t textbgcolor = cBlack;
uint16_t textsize_x = 1;
uint16_t textsize_y = 1;
uint16_t wrap = 0;
void TextPos(uint16_t x, uint16_t y)
{
  cursor_x = x;
  cursor_y = y;
}
uint16_t writechar(uint8_t c)
{
  if (c == '\n')
  {
    cursor_x = 0;
    cursor_y += (int16_t)textsize_y * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
  }
  else if (c != '\r')
  {
    uint8_t first = pgm_read_byte(&gfxFont->first);
    if ((c >= first) && (c <= (uint8_t)pgm_read_byte(&gfxFont->last)))
    {
      GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c - first);
      uint8_t w = pgm_read_byte(&glyph->width), h = pgm_read_byte(&glyph->height);
      if ((w > 0) && (h > 0))
      {                                                      // Is there an associated bitmap?
        int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset); // sic
        if (wrap && ((cursor_x + textsize_x * (xo + w)) > _width))
        {
          cursor_x = 0;
          cursor_y += (int16_t)textsize_y * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
        }
        drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize_x, textsize_y);
      }
      cursor_x += (uint8_t)pgm_read_byte(&glyph->xAdvance) * (int16_t)textsize_x;
    }
  }
  return 1;
}

void writestring(uint8_t *s)
{
  while (*s)
    writechar(*s++);
}

void writef(char *format, ...)
{
  uint16_t ocx = cursor_x;
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, 255, format, args);
  writestring(buffer);
  va_end(args);
  cursor_y += (int16_t)textsize_y * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
  cursor_x = ocx;
}

void HLineBlend(int x, int y, int w, uint16_t col)
{
  if (y >= _height)
    return;
  if (x >= _width)
    return;

  if ((x + w) >= _width)
    w = (_width - x);
  const int bmask1 = 0b0000011111100000;
  const int bmask2 = 0b1111100000011111;
  for (int a = 0; a < w; a++)
  {
    uint32_t pix = (buf[x + (y * _width) + a]);
    pix = eswap(pix);
    uint32_t pix1 = pix & bmask1;
    uint32_t pix2 = pix & bmask2;
    uint32_t col1 = (col)&bmask1;
    uint32_t col2 = (col)&bmask2;
    pix = (((pix1 + col1) >> 1) & bmask1) | (((pix2 + col2) >> 1) & bmask2);
    buf[x + (y * _width) + a] = eswap(pix);
  }
}

void HLine(int x, int y, int w, uint16_t col)
{
  if (y >= _height)
    return;
  if (x >= _width)
    return;

  if ((x + w) >= _width)
    w = (_width - x);
  for (int a = 0; a < w; a++)
    buf[x + (y * _width) + a] = eswap(col);
}

void VLine(int x, int y, int w, uint16_t col)
{
  if (y >= _height)
    return;
  if (x >= _width)
    return;

  if ((y + w) >= _height)
    w = (_height - y);
  for (int a = 0; a < w; a++)
    buf[x + ((y + a) * _width)] = eswap(col);
}

void FillRect(int x, int y, int w, int h, uint16_t col)
{
  for (int a = 0; a < h; a++)
    HLine(x, y + a, w, col);
}

void FillRectBlend(int x, int y, int w, int h, uint16_t col)
{
  for (int a = 0; a < h; a++)
    HLineBlend(x, y + a, w, col);
}
// Display Thread
int DRP = 0;
int DWP = 0;
static volatile uint8_t Go = 0;
static volatile uint64_t spitime = 0;
void DisplayThread()
{
  while (1)
  {
    while (Go == 0)
      ;
    SetWindow(0, 0, 240, 240);
    StartPixels();
    uint64_t st = time_us_64();
    spi_write_blocking(SPI_PORT, (uint8_t *)buf, _width * _height * 2);
    spitime = (time_us_64() - st);
    Go = 0;
  }
}

void gset_sys_clock_pll(uint32_t vco_freq, uint post_div1, uint post_div2)
{
  if (!running_on_fpga())
  {
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    pll_init(pll_sys, 1, vco_freq, post_div1, post_div2);
    uint32_t freq = vco_freq / (post_div1 * post_div2);

    // Configure clocks
    // CLK_REF = XOSC (12MHz) / 1 = 12MHz
    clock_configure(clk_ref,
                    CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
                    0, // No aux mux
                    12 * MHZ,
                    12 * MHZ);

    // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    freq, freq);

    //    clock_configure(clk_peri,
    //                    0, // Only AUX mux on ADC
    //                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
    //                    48 * MHZ,
    //                    48 * MHZ);
  }
}

static inline bool gset_sys_clock_khz(uint32_t freq_khz, bool required)
{
  uint vco, postdiv1, postdiv2;
  if (check_sys_clock_khz(freq_khz, &vco, &postdiv1, &postdiv2))
  {
    gset_sys_clock_pll(vco, postdiv1, postdiv2);
    return true;
  }
  else if (required)
  {
    panic("System clock of %u kHz cannot be exactly achieved", freq_khz);
  }
  return false;
}

float ReadTemp()
{
  uint16_t raw = adc_read();
  const float conversion_factor = 3.3f / (1 << 12);
  float result = raw * conversion_factor;
  return 27 - (result - 0.706) / 0.001721;
}
#define cpuspd 170000
#define spispd 80000
int main()
{

  sleep_ms(10);

  stdio_init_all();
  WriteFlash();
  /*
   clock_configure(clk_peri,
                        0, // Only AUX mux on ADC
                        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                        48*3 * MHZ,
                        48*3 * MHZ);
*/
  // Configure ADC
  adc_init();
  adc_set_temp_sensor_enabled(true);
  adc_select_input(4);

  PIO pio = pio0;
  uint sm = 0;
  uint offset = 0; //pio_add_program(pio, &st7789_lcd_program);
                   //  st7789_lcd_program_init(pio, sm, offset, PIN_DIN, PIN_CLK, SERIAL_CLK_DIV);

  spi_init(SPI_PORT, spispd * 1000);

  spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);
  uint32_t spibaud = spi_set_baudrate(SPI_PORT, spispd * 1000);

  gpio_set_function(PIN_CLK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_DIN, GPIO_FUNC_SPI);

  gpio_init(PIN_DC);
  gpio_init(PIN_RESET);
  gpio_init(PIN_CS);

  gpio_set_dir(PIN_DC, GPIO_OUT);
  gpio_set_dir(PIN_RESET, GPIO_OUT);
  gpio_set_dir(PIN_CS, GPIO_OUT);

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  HardReset();
  lcd_init(pio, sm, st7789_init_seq);

  //vreg_set_voltage(VREG_VOLTAGE_1_30);
  //sleep_ms(100);

  gset_sys_clock_khz(cpuspd, false);


      //  clock_configure(clk_peri,
      //                  0, // Only AUX mux on ADC
      //                  CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
      //                  24 * MHZ,
      //                  24 * MHZ);

  uint16_t *FlashPic = (uint16_t *)FlashRam;
  // gpio_put(PIN_BL, 1);

  // Other SDKs: static image on screen, lame, boring
  // Raspberry Pi Pico SDK: spinning image on screen, bold, exciting

  // Lane 0 will be u coords (bits 8:1 of addr offset), lane 1 will be v
  // coords (bits 16:9 of addr offset), and we'll represent coords with
  // 16.16 fixed point. ACCUM0,1 will contain current coord, BASE0/1 will
  // contain increment vector, and BASE2 will contain image base pointer
#define UNIT_LSB 16
  interp_config lane0_cfg = interp_default_config();
  interp_config_set_shift(&lane0_cfg, UNIT_LSB - 1); // -1 because 2 bytes per pixel
  interp_config_set_mask(&lane0_cfg, 1, 1 + (LOG_IMAGE_SIZE - 1));
  interp_config_set_add_raw(&lane0_cfg, true); // Add full accumulator to base with each POP
  interp_config lane1_cfg = interp_default_config();
  interp_config_set_shift(&lane1_cfg, UNIT_LSB - (1 + LOG_IMAGE_SIZE));
  interp_config_set_mask(&lane1_cfg, 1 + LOG_IMAGE_SIZE, 1 + (2 * LOG_IMAGE_SIZE - 1));
  interp_config_set_add_raw(&lane1_cfg, true);

  interp_set_config(interp0, 0, &lane0_cfg);
  interp_set_config(interp0, 1, &lane1_cfg);
  interp0->base[2] = (uint32_t)FlashPic; //raspberry_256x256;
//interp0->base[2] = (uint32_t)raspberry_256x256;

  float theta = 0.f;
  float theta_max = 2.f * (float)M_PI;
  // Clear the framebuffer
  for (int x = 0; x < _width * _height; ++x)
    buf[x] = 0x000;

  /*
    for (int x = 0; x < 320; ++x) 
        buf[x]=0x000;
    for (int y = 0; y < SCREEN_HEIGHT; ++y) 
        {
   
            SetWindow(0,y,240,240);
            spi_write_blocking(SPI_PORT,(uint8_t*) buf, 320*2);          
        }
*/
//spi_deinit(SPI_PORT);
//spi_set_format(SPI_PORT, 16, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);
//   spibaud = spi_set_baudrate(SPI_PORT, spispd * 1000);

  SetOrientation(2);
  SetWindow(0, 0, _width, _height);
  StartPixels();
  //multicore_launch_core1(DisplayThread);
  uint16_t *Swapit = (uint16_t *)raspberry_256x256;
  for (int a = 0; a < 256 * 256; a++)
    Swapit[a] = eswap(Swapit[a]);

  while (1)
  {
    uint64_t st = time_us_64();
    uint64_t fst=st;
    theta += 0.02f;
    if (theta > theta_max)
      theta -= theta_max;
    float zoom = 0.25f + (1.0f + cosf(theta * 0.25f));
    int32_t rotate[4] =
        {
            (zoom * cosf(theta)) * (1 << UNIT_LSB), (-zoom * sinf(theta)) * (1 << UNIT_LSB),
            (zoom * sinf(theta)) * (1 << UNIT_LSB), (zoom * cosf(theta)) * (1 << UNIT_LSB)};
    interp0->base[0] = rotate[0];
    interp0->base[1] = rotate[2];
    // st7789_start_pixels(pio, sm);
    int cc = 0;
    static int yscan = 0;

    for (int y = 0; y < _height; ++y)
    {
      interp0->accum[0] = rotate[1] * y;
      interp0->accum[1] = rotate[3] * y;
      for (int x = 0; x < _width; ++x)
      {
        uint16_t colour = *(uint16_t *)(interp0->pop[2]);
        // st7789_lcd_put(pio, sm, colour >> 8);
        //  st7789_lcd_put(pio, sm, colour & 0xff);
        buf[cc++] = colour;
      }
    }
    // 4ms @125mhz
    static uint64_t btime = 0;
    btime = (time_us_64() - st);
    static int sy = 60;

    // RGB Bars
    //for (int a=0;a<64*2;a++)
    //{
    // HLine(40,a,50,a/2);
    // HLine(40+50,a,50,(a/2)<<5);
    // HLine(40+50*2,a,50,(a/2)<<11);
    //}
    // Text
        st = time_us_64();
    FillRectBlend(5, 6, 230, 230, 0);
    textcolor = cWhite;
    TextPos(10, 25);
    writef("CurX %d CurY %d", cursor_x, cursor_y);
    textcolor = cYellow;
    writef("Render %2.1f ms", (double)btime / 1000);
    textcolor = cYellow;
    writef("Spi Blit %2.1f ms", (double)spitime / 1000);
    textcolor = cYellow;
    writef("Spi Baud %2.1f mhz", (double)spibaud / 1000000);
    textcolor = cYellow;
    writef("Chip Temp %2.1fc", ReadTemp());
    writef("CPU speed %dMhz", cpuspd / 1000);
    static int uitime=0;
    uitime=time_us_64()-st;
    writef("UITime %2.1f ms",(double)uitime/1000);
    
    static int frametime=0;
    writef("Frame %2.1fms %2.1ffps",(double)frametime/1000,1000.f/((double)frametime/1000));
    // VSync Bar
    static int vbar = 0;
    VLine(60, 200, 40, cYellow);
    VLine(61, 200, 40, cYellow);

    VLine(vbar, 200, 40, cRed);
    VLine(vbar + 1, 200, 40, cRed);

    vbar = (vbar + 1) % 60;

    st = time_us_64();
    SetWindow(0, 0, _width, _height);
    //  StartPixels();
    spi_write_blocking(SPI_PORT, (uint8_t *)buf, _width * _height * 2);
    //spi_write16_blocking(SPI_PORT, (uint16_t *)buf, _width * _height);
    spitime = (time_us_64() - st);
frametime=time_us_64()-fst;
    static int xx = 0;

    if (xx == 0)
      gpio_put(LED_PIN, 1);
    if (xx == 10)
      gpio_put(LED_PIN, 0);
    xx++;
    if (xx >= 60)
      xx = 0;
  }
}