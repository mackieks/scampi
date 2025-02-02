/*
  Scampi firmware

  Application code for ATtiny1616 and TLV320AIC3110

  https://www.ti.com/lit/ds/symlink/tlv320aic3110.pdf

  Mackie Kannard-Smith, James Smith 2024
  MIT License

*/

#include <stdint.h>
#include <string.h>

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "gpio.h"
#include "i2c.h"
#include "i2c/tlv320aic3110.h"

// speaker type (controls gain)
#define IPHONE_15PM 1
#define CMS_151135 0
#define FOUR_OHM_20MM 0

static volatile bool analog_vol_ctrl   = 0;
static volatile bool headset_connected = 0;
static volatile bool mute              = 0;
static volatile bool vol_down_pressed  = 0;
static volatile bool vol_up_pressed    = 0;

// higher value = reduced gain (see Table 7-38 in datasheet)
static const uint8_t min_vol = 118;
static const uint8_t max_vol = 0;

#ifdef IPHONE_15PM
static const uint8_t spk_gain = 0x30; // +24dB
#elif CMS_151135
static const uint8_t spk_gain = 0x20; // +16dB
#elif FOUR_OHM_20MM
static const uint8_t spk_gain = 0x28; // +20dB
#endif

static const uint8_t hp_gain = 0x00; // 0dB (suitable for my 15Ω headphones)

static volatile uint8_t spk_vol = 16;
static volatile uint8_t hp_vol  = 10;

// GPIO pin definitions
static const gpio_t RESET  = {&PORTB, 2}; // PB2
static const gpio_t VOL_UP = {&PORTC, 1}; // PC1
static const gpio_t VOL_DN = {&PORTC, 0}; // PC0
static const gpio_t A_MODE = {&PORTC, 2}; // PC2 (analog vol ctrl mode)

static const gpio_t MODE0 = {&PORTA, 4}; // PA4
static const gpio_t MODE1 = {&PORTA, 5}; // PA5
static const gpio_t MODE2 = {&PORTA, 6}; // PA6
static const gpio_t MODE3 = {&PORTA, 7}; // PA7

// Initialize the GPIO pins
static void gpio_init()
{
  // Codec reset, active low
  gpio_output(RESET);
  gpio_set_low(RESET);

  gpio_input(VOL_UP);
  gpio_input(VOL_DN);
  gpio_input(A_MODE);
  gpio_input(MODE0);
  gpio_input(MODE1);
  gpio_input(MODE2);
  gpio_input(MODE3);

  if (gpio_read(A_MODE)) { // enable analog volume control mode

    analog_vol_ctrl = true;

    // ADC on VOL_DN (PC0)
    ADC1.CTRLA   = 0x07; // enable ADC in free-running 8-bit mode
    ADC1.CTRLC   = 0x01010000; // reduced capacitance mode, set VREF to VDD
    ADC1.MUXPOS  = ADC_MUXPOS_AIN6_gc; // select PC0
    ADC1.COMMAND = 0b1; // start conversion
  } else {
    gpio_pullup(VOL_DN);
  }

  // internal pullups for IO pins
  gpio_pullup(VOL_UP);
  gpio_pullup(A_MODE);
  gpio_pullup(MODE0);
  gpio_pullup(MODE1);
  gpio_pullup(MODE2);
  gpio_pullup(MODE3);
}

static uint8_t mode_detect()
{
  uint8_t mode = 0;

  mode |= !gpio_read(MODE0);
  mode |= !gpio_read(MODE1) << 1;
  mode |= !gpio_read(MODE2) << 2;
  mode |= !gpio_read(MODE3) << 3;

  return (mode);
}

static void codec_write(int reg, int value)
{
  i2c_reg_write_byte(TLV320_ADDR, reg, value);
}

volatile int codec_read(int reg)
{
  int data;
  i2c_reg_read_byte(TLV320_ADDR, reg, &data);
  return (data);
}

static void codec_reset()
{
  // Reset codec
  gpio_set_low(RESET);
  _delay_ms(10);
  gpio_set_high(RESET);
}

static void set_dac_gain(uint8_t gain)
{
  // digital gain
  codec_write(0x00, 0x00); // select page 0
  codec_write(0x41, gain); // DAC -> +24dB gain left
  codec_write(0x42, gain); // DAC -> +24dB gain right
}

static void set_spk_vol(uint8_t vol)
{
  codec_write(0x00, 0x01); // Select page 1
  codec_write(0x26, spk_vol); // SPL analog volume
  codec_write(0x27, spk_vol); // SPR analog volume
}

static void set_hp_vol(uint8_t vol)
{
  codec_write(0x00, 0x01); // Select page 1
  codec_write(0x24, hp_vol); // HPL analog volume
  codec_write(0x25, hp_vol); // HPR analog volumee
}

static void mute_spk()
{
  codec_write(0x00, 0x01); // Select page 1
  codec_write(0x2A, 0x00); // SPL driver mute
  codec_write(0x2B, 0x00); // SPR driver mute
  // codec_write(0x20, 0x06); // power down class D drivers
}

static void mute_hp()
{
  codec_write(0x00, 0x01); // Select page 1
  codec_write(0x28, 0x02); // HPL driver mute
  codec_write(0x29, 0x02); // HPR driver mute
  // codec_write(0x1F, 0x04); // power down headphone drivers
}

static void unmute_spk()
{
  codec_write(0x00, 0x01); // Select page 1
  codec_write(0x20, 0xC6); // power up class D drivers
  codec_write(0x2A, 0x04); // SPL driver unmute
  codec_write(0x2B, 0x04); // SPR driver unmute
}

static void unmute_hp()
{
  codec_write(0x00, 0x01); // Select page 1
  codec_write(0x1F, 0xC4); // power up headphone drivers
  codec_write(0x28, 0x06); // HPL driver unmute
  codec_write(0x29, 0x06); // HPR driver unmute
}

static void toggle_mute()
{
  mute = !mute;
  if (mute) {
    if (headset_connected)
      mute_hp();
    else
      mute_spk();
  } else {
    if (headset_connected)
      unmute_hp();
    else
      unmute_spk();
  }
}

/* checks for headphones and handles muting/unmuting */
static void check_headphones()
{
  // headphone detect
  codec_write(0x00, 0x00); // select page 0
  if (codec_read(0x43) & 1 << 5) { // if headset is connected...
    mute_spk();
    set_dac_gain(hp_gain);
    unmute_hp();
    headset_connected = true;
  } else { // if headset is not connected...
    mute_hp();
    set_dac_gain(spk_gain);
    unmute_spk();
    headset_connected = false;
  }
}

/* polls volume buttons */
static void check_buttons()
{
  vol_down_pressed = !gpio_read(VOL_DN);
  vol_up_pressed   = !gpio_read(VOL_UP);
}

int main(void)
{
  // Enable prescaler, and set prescaler division to 4 to run at 5MHz
  CPU_CCP           = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm;

  // Enable interrupts
  sei();

  // Initalize the GPIOs
  gpio_init();

  // Initialize as an I2C controller
  i2c_configure(I2C_MODE_STANDARD);

  codec_reset();

  // Initialize TLV320AIC3110
  codec_write(0x0, 0x0); //  select page 0
  codec_write(0x01, 0x01); // soft reset
  _delay_ms(10);

  uint8_t mode = mode_detect();

  mode = 0b0001;

  switch (mode) {
    case 0b0000:
      // analog mode
      codec_write(0x3F, 0b00000010); // power down DACs and disable data paths
      break;

    case 0b0001:
      // SNES mode
      codec_write(0x04, 0x00); // PLL_CLKIN = MCLK, CODEC_CLKIN = MCLK
      codec_write(0x05, 0x00); // PLL powered off.

      codec_write(0x1B, 0x80); // right justified, 16 bit
      codec_write(0x0B, 0x81); // NDAC is powered and set to 1
      codec_write(0x0C, 0x82); // MDAC is powered and set to 2
      break;

    case 0b0010:
      // N64 mode
      codec_write(0x04, 0b00000111); // PLL_CLKIN = BCLK, CODEC_CLKIN = PLL_CLK
      codec_write(0x05, 0b10010011); // PLL power up

      codec_write(0x1B, 0x80); // right justified, 16 bit
      codec_write(0x0B, 0x84); // NDAC is powered and set to 1
      codec_write(0x0C, 0x84); // MDAC is powered and set to 2
      break;

    case 0b0011:
      // GC mode
      break;

    case 0b0100:
      // Wii mode
      codec_write(0x04, 0x00); // PLL_CLKIN = MCLK, CODEC_CLKIN = MCLK
      codec_write(0x05, 0x00); // PLL powered off.

      codec_write(0x1B, 0xF0); // supposed to be left justified 32-bit
      codec_write(0x0B, 0x81); // NDAC is powered and set to 1
      codec_write(0x0C, 0x82); // MDAC is powered and set to 2
      break;

    case 0b0101:
      // Wii U mode
      // same format as Wii but without MCLK?
      codec_write(0x04, 0b00000111); // PLL_CLKIN = BCLK, CODEC_CLKIN = PLL_CLK
      codec_write(0x05, 0b10010100); // PLL powered up. P = 1, R = 4
      codec_write(0x06, 0b00000111); // J = 7

      codec_write(0x1B, 0xF0); // supposed to be left justified 32-bit
      codec_write(0x0B, 0b10000010); // NDAC powered up. NDAC = 2
      codec_write(0x0C, 0b10000111); // MDAC powered up. MDAC = 7
      break;

    case 0b0110:
      // Saturn mode
      break;

    case 0b0111:
      // Dreamcast mode (same as SNES)
      codec_write(0x04, 0x00); // PLL_CLKIN = MCLK, CODEC_CLKIN = MCLK
      codec_write(0x05, 0x00); // PLL powered off.

      codec_write(0x1B, 0x80); // right justified, 16 bit
      codec_write(0x0B, 0x81); // NDAC is powered and set to 1
      codec_write(0x0C, 0x82); // MDAC is powered and set to 2
      break;

    case 0b1000:
      // PS1 mode
      break;

    case 0b1001:
      // PS2 mode
      break;

    case 0b1010:
      // PS3 mode
      break;

    case 0b1011:
      // XBOX mode
      break;

    case 0b1100:
      // XBOX360 mode
      break;

    case 0b1101:
      // Jaguar mode
      break;

    case 0b1110:
      // NEOGEO mode
      break;

    case 0b1111:
      // ???? mode
      // Wii mode
      codec_write(0x04, 0x00); // PLL_CLKIN = MCLK, CODEC_CLKIN = MCLK
      codec_write(0x05, 0x00); // PLL powered off.

      codec_write(0x1B, 0xF0); // supposed to be left justified 32-bit
      codec_write(0x0B, 0x81); // NDAC is powered and set to 1
      codec_write(0x0C, 0x82); // MDAC is powered and set to 2
      break;

    default:
      // ????
      break;
  }

  // finish up amp init

  if (mode == 0) { // analog mode
    codec_write(0x23, 0b00100010); // analog inputs routed to mixers
  } else { // digital modes
    codec_write(0x74, 0x00); // DAC -> volume control pin disable
    codec_write(0x44, 0x00); // DAC -> DRC disable

    codec_write(0x00, 0x01); // Select page 1

    codec_write(0x23, 0x44); // DAC outputs routed to mixers

    codec_write(0x00, 0x00); // select page 0

    codec_write(0x3F, 0xD6); // power up DACs and enable data paths
    codec_write(0x40, 0x00); // unmute DAC left and right channels
  }

  codec_write(0x00, 0x01); // Select page 1
  codec_write(0x21, 0x4E); // De-pop, power on = 800 ms, step time = 4ms

  set_hp_vol(hp_vol);
  set_spk_vol(spk_vol);

  // mute everything at first
  mute_hp();
  mute_spk();

  codec_write(0x2E, 0x0B); // Micbias set to AVDD (for HPS pull-up)

  codec_write(0x00, 0x00); // select page 0

  codec_write(0x43, 0x93); // headset detection enabled

  check_headphones();

  // Main loop
  while (1) {

    if (!mute)
      check_headphones();

    if (analog_vol_ctrl) {
      // read analog volume wheel
    } else { // digital volume control
      check_buttons();
      if (vol_down_pressed) {
        if (vol_up_pressed) { // both vol buttons pressed. MUTE/UNMUTE
          toggle_mute();
          _delay_ms(50);
          while (vol_down_pressed && vol_up_pressed) { check_buttons(); } // wait for buttons to be released
        } else { // decrease volume
          if (!mute) {
            if (headset_connected) {
              if (hp_vol < min_vol) {
                hp_vol += 2;
                set_hp_vol(hp_vol);
              }
            } else {
              if (spk_vol < min_vol) {
                spk_vol += 2;
                set_spk_vol(spk_vol);
              }
            }
          }
        }

      } else if (vol_up_pressed) {
        if (vol_down_pressed) { // both vol buttons pressed. MUTE/UNMUTE
          toggle_mute();
          _delay_ms(50);
          while (vol_down_pressed && vol_up_pressed) { check_buttons(); } // wait for buttons to be released
        } else { // increase volume
          if (!mute) {
            if (headset_connected) {
              if (hp_vol > max_vol) {
                hp_vol -= 2;
                set_hp_vol(hp_vol);
              }
            } else {
              if (spk_vol > max_vol) {
                spk_vol -= 2;
                set_spk_vol(spk_vol);
              }
            }
          }
        }
      }
    }

    _delay_ms(50);
  }
}