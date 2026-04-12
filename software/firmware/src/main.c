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

// old PCB has A_MODE tied to GND.
#define FORCE_AMODE 0

// speaker type (controls gain)
#define IPHONE_15PM 0
#define SWITCH_2 1
#define CMS_151135 0
#define FOUR_OHM_20MM 0

// set this to 1 to change the volume pot rotation direction
#define INVERT_VOL_POT 0

// RTC millisecond counter
static volatile uint32_t millis = 0;

static volatile uint8_t mode = 0;

// set this to the min and max ADC readings you get from your volume pot
// for example, if Vmin = 100mV, (0.100 / 3.3) * 1024 = 31
static volatile uint8_t VOL_POT_MIN    = 128; // real value comes from eeprom
static volatile uint8_t VOL_POT_MAX    = 128; // real value comes from eeprom
static volatile uint8_t MUTE_THRESHOLD = 1; // Adjustable threshold above/below POT_MIN/POT_MAX to mute at

static volatile bool analog_vol_ctrl   = 0;
static volatile bool headset_connected = 0;
static volatile bool mute              = 0;
static volatile bool vol_down_pressed  = 0;
static volatile bool vol_up_pressed    = 0;

// higher value = reduced gain (see Table 7-38 in datasheet)
static const uint8_t min_vol = 50;
static const uint8_t max_vol = 0;

#if IPHONE_15PM
static const uint8_t spk_gain = 0x0; // +24dB
#elif SWITCH_2
static const uint8_t spk_gain = 0x0; // +20dB
#elif CMS_151135
static const uint8_t spk_gain = 0x20; // +16dB
#elif FOUR_OHM_20MM
static const uint8_t spk_gain = 0x28; // +20dB
#endif

static const uint8_t hp_gain = 0x00; // 0dB (suitable for my 15Ω headphones)

static volatile uint8_t vol     = 0;
static volatile uint8_t spk_vol = 0;
static volatile uint8_t hp_vol  = 0;

static volatile uint8_t vol_pot = 0; // analog volume pot reading (0 - 255)

// GPIO pin definitions
static const gpio_t RESET  = {&PORTB, 2}; // PB2
static const gpio_t VOL_UP = {&PORTC, 1}; // PC1
static const gpio_t VOL_DN = {&PORTC, 0}; // PC0
static const gpio_t A_MODE = {&PORTC, 2}; // PC2 (analog vol ctrl mode)

static const gpio_t MODE0 = {&PORTA, 4}; // PA4
static const gpio_t MODE1 = {&PORTA, 5}; // PA5
static const gpio_t MODE2 = {&PORTA, 6}; // PA6
static const gpio_t MODE3 = {&PORTA, 7}; // PA7

// Initialize the RTC for periodic interrupts
void rtc_init()
{
  RTC.CLKSEL     = RTC_CLKSEL_INT32K_gc;
  RTC.PITINTCTRL = RTC_PI_bm;
  RTC.PITCTRLA   = RTC_PERIOD_CYC32_gc | RTC_PITEN_bm;
}

// Handle periodic RTC interrupts (every ~1ms)
ISR(RTC_PIT_vect)
{
  // Clear the interrupt flag
  RTC.PITINTFLAGS = RTC_PI_bm;

  // Update the "milliseconds since boot" counter
  millis++;
}

// Force reset eeprom for debug
static void reset_eeprom()
{
  // Write the default pot min and max values
  eeprom_write_byte(0x00, 0x80); // VOL_POT_MIN
  eeprom_write_byte(0x01, 0x80); // VOL_POT_MAX
}

// Initialize the EEPROM if it has never been initialized
static void eeprom_init()
{
  // Return early if the EEPROM has already been initialized
  if (eeprom_read_word(0xFE) == 0xCAFE)
    return;

  // Write the default pot min and max values
  eeprom_write_byte(0x00, 0x80); // VOL_POT_MIN
  eeprom_write_byte(0x01, 0x80); // VOL_POT_MAX

  // Write the signature
  eeprom_write_word(0xFE, 0xCAFE);
}

// Initialize the GPIO pins
static void gpio_init()
{
  // Codec reset, active low
  gpio_output(RESET);
  gpio_set_low(RESET);

  gpio_input(VOL_DN);
  gpio_input(A_MODE);
  gpio_input(MODE0);
  gpio_input(MODE1);
  gpio_input(MODE2);
  gpio_input(MODE3);

  // internal pullups for IO pins

  gpio_pullup(A_MODE);
  gpio_pullup(MODE0);
  gpio_pullup(MODE1);
  gpio_pullup(MODE2);
  gpio_pullup(MODE3);

  if (FORCE_AMODE ? true : !gpio_read(A_MODE)) { // enable analog volume control mode

    analog_vol_ctrl = true;

    // ADC1 on VOL_DN (PC0)
    ADC1.CTRLA   = 0x07; // enable ADC in free-running 8-bit mode
    ADC1.CTRLC   = 0x01010011; // reduced capacitance mode, set VREF to VDD, div16 prescaler
    ADC1.MUXPOS  = ADC_MUXPOS_AIN6_gc; // select PC0
    ADC1.COMMAND = 0b1; // start conversion

    gpio_output(VOL_UP); // use VOL_UP to power the volume pot
    gpio_set_high(VOL_UP);

    // grab VOL_POT_MIN and VOL_POT_MAX from EEPROM
    VOL_POT_MIN = eeprom_read_byte(0x00);
    VOL_POT_MAX = eeprom_read_byte(0x01);

  } else {
    gpio_input(VOL_UP); // use VOL_UP for a button
    gpio_pullup(VOL_UP);
    gpio_pullup(VOL_DN);
  }
}

static uint8_t mode_detect()
{

  mode |= !gpio_read(MODE0);
  mode |= !gpio_read(MODE1) << 1;
  mode |= !gpio_read(MODE2) << 2;
  mode |= !gpio_read(MODE3) << 3;

  return (mode);
}

static void codec_write(int reg, int value)
{ i2c_reg_write_byte(TLV320_ADDR, reg, value); }

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

  if (mode == 0) { // analog input mode needs higher SPx driver gain
    codec_write(0x2A, 0b00011100); // SPL driver unmute, +24dB gain
    codec_write(0x2B, 0b00011100); // SPR driver unmute, +24dB gain
  } else { // digital input modes use digital volume control
    codec_write(0x2A, 0b00011100); // SPL driver unmute, +6dB gain
    codec_write(0x2B, 0b00011100); // SPR driver unmute, +6dB gain
  }
}

static void unmute_hp()
{
  codec_write(0x00, 0x01); // Select page 1
  codec_write(0x1F, 0xC4); // power up headphone drivers

  if (mode == 0) { // analog input mode needs higher SPx driver gain
    codec_write(0x28, 0b00101110); // HPL driver unmute, +8dB gain
    codec_write(0x29, 0b00101110); // HPR driver unmute, +8dB gain
  } else { // digital input modes use digital volume control
    codec_write(0x28, 0b00001110); // HPL driver unmute, +2dB gain
    codec_write(0x29, 0b00001110); // HPR driver unmute, +2dB gain
  }
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

/* reads analog volume pot and performs wheel limit calibration */
static void read_pot()
{
  vol_pot = ADC1.RESL;

  // run wheel limit calibration for first 3s after power-on
  if (millis < 5000) {
    if (vol_pot < VOL_POT_MIN) {
      VOL_POT_MIN = vol_pot;
      eeprom_update_byte(0x00, VOL_POT_MIN);
    }
    if (vol_pot > VOL_POT_MAX) {
      VOL_POT_MAX = vol_pot;
      eeprom_update_byte(0x01, VOL_POT_MAX);
    }
  } else {
    // catch miscalibration
    if (vol_pot > VOL_POT_MAX)
      vol_pot = VOL_POT_MAX;
    if (vol_pot < VOL_POT_MIN)
      vol_pot = VOL_POT_MIN;
  }
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{ return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }

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

  // reset_eeprom for debug
  // reset_eeprom();

  // Initialize eeprom if first boot
  eeprom_init();

  // Initialize the RTC (millis timer)
  rtc_init();

  // Initalize the GPIOs
  gpio_init();

  // Initialize as an I2C controller
  i2c_configure(I2C_MODE_STANDARD);

  codec_reset();

  // Initialize TLV320AIC3110
  codec_write(0x0, 0x0); //  select page 0
  codec_write(0x01, 0x01); // soft reset
  _delay_ms(10);

  // Set TLV320AIC3110 timer to use internal oscillator
  // Required for headphone detection to work without MCLK
  codec_write(0x0, 0x3); //  select page 3
  codec_write(0x10, 0x08); // select internal oscillator

  mode = mode_detect();
  codec_write(0x0, 0x0); //  select page 0

  switch (mode) {

      // SNES, Saturn, Dreamcast, and PS! all work with the same amp config. However, since their digital signal levels
      // are different, I'm keeping separate modes for each console. This lets me dial in the DAC gain for each console
      // without more compile-time flags.

    case 0b0000:
      // analog mode
      codec_write(0x3F, 0b00000010); // power down DACs and disable data paths
      break;

    case 0b0001:

      // SNES mode
      // • 24-bit frame size, 16-bit word size, right-justified
      // • W (WS / LRCLK / FS): 32kHz
      // • C (SCK / BCLK): 1.536MHz (32kHz * 2ch * 24 bit frame size)
      // • M (MCLK): 8.192MHz (256Fs = 256 * 32kHz)

      codec_write(0x04, 0x00); // PLL_CLKIN = MCLK, CODEC_CLKIN = MCLK
      codec_write(0x05, 0x00); // PLL powered off.

      codec_write(0x1B, 0x80); // right justified, 16 bit
      codec_write(0x0B, 0x81); // NDAC is powered and set to 1
      codec_write(0x0C, 0x82); // MDAC is powered and set to 2
      break;

    case 0b0010:
      // N64 mode
      // • 16-bit frame size, 16-bit word size, right-justified
      // • W (WS / LRCLK / FS): 48kHz
      // • C (SCK / BCLK): 1.536MHz (48kHz * 2ch * 16 bit frame size)
      // • no MCLK
      // TODO: apparently the sample rate changes from 32kHz to 22kHz in 1080 Snowboarding 2P mode

      codec_write(0x04, 0b00000111); // PLL_CLKIN = BCLK, CODEC_CLKIN = PLL_CLK
      codec_write(0x05, 0b10010011); // PLL powered up. P = 1, R = 3.
      // J defaults to 4

      codec_write(0x1B, 0x80); // right justified, 16 bit
      codec_write(0x0B, 0x84); // NDAC is powered and set to 4
      codec_write(0x0C, 0x84); // MDAC is powered and set to 4

      // to test (384kHz oversampling)
      // codec_write(0x04, 0b00000111); // PLL_CLKIN = BCLK, CODEC_CLKIN = PLL_CLK
      // codec_write(0x05, 0b10010000); // PLL powered up. P = 1, R = 16
      // // J defaults to 4

      // codec_write(0x1B, 0x80); // right justified, 16 bit
      // codec_write(0x0B, 0x82); // NDAC is powered and set to 2
      // codec_write(0x0C, 0x81); // MDAC is powered and set to 1
      break;

    case 0b0011:
      // GC mode
      // • 16-bit frame size, 16-bit word size, right-justified
      // • W (WS / LRCLK / FS): 48kHz
      // • C (SCK / BCLK): 1.536MHz (48kHz * 2ch * 16 bit frame size)
      // • no MCLK

      codec_write(0x04, 0b00000111); // PLL_CLKIN = BCLK, CODEC_CLKIN = PLL_CLK
      codec_write(0x05, 0b10010010); // PLL powered up. P = 1, R = 2
      codec_write(0x06, 0b00100000); // J = 32 for 2x oversampling

      codec_write(0x1B, 0x80); // right justified, 16 bit
      codec_write(0x0B, 0x84); // NDAC is powered and set to 4
      codec_write(0x0C, 0x82); // MDAC is powered and set to 2
      break;

    case 0b0100:
      // Wii mode
      // • 32-bit frame size, 32-bit word size, left-justified
      // • W (WS / LRCLK / FS): 48kHz
      // • C (SCK / BCLK): 3.072MHz (48kHz * 2ch * 32 bit frame size)
      // • M (MCLK): 12.228MHz (256Fs = 256 * 48kHz)

      codec_write(0x04, 0x00); // PLL_CLKIN = MCLK, CODEC_CLKIN = MCLK
      codec_write(0x05, 0x00); // PLL powered off.

      codec_write(0x1B, 0xF0); // supposed to be left justified 32-bit
      codec_write(0x0B, 0x81); // NDAC is powered and set to 1
      codec_write(0x0C, 0x82); // MDAC is powered and set to 2
      break;

    case 0b0101:
      // Wii U mode
      // • 32-bit frame size, 32-bit word size, left-justified
      // • W (WS / LRCLK / FS): 48kHz
      // • C (SCK / BCLK): 3.072MHz (48kHz * 2ch * 32 bit frame size)
      // • no MCLK

      codec_write(0x04, 0b00000111); // PLL_CLKIN = BCLK, CODEC_CLKIN = PLL_CLK
      codec_write(0x05, 0b10010010); // PLL powered up. P = 1, R = 2
      codec_write(0x06, 0b00010000); // J = 16 for 2x oversampling

      codec_write(0x1B, 0xF0); // supposed to be left justified 32-bit
      codec_write(0x0B, 0x84); // NDAC is powered and set to 4
      codec_write(0x0C, 0x82); // MDAC is powered and set to 2
      break;

    case 0b0110:
      // Saturn mode
      // • 32-bit frame size, 16-bit word size, right-justified
      // • W (WS / LRCLK / FS): 44.1kHz
      // • C (SCK / BCLK): 2.8224MHz (44.1kHz * 2ch * 32 bit frame size)
      // • M (MCLK): 11.2896 (256Fs = 256 * 44.1kHz)

      codec_write(0x04, 0x00); // PLL_CLKIN = MCLK, CODEC_CLKIN = MCLK
      codec_write(0x05, 0x00); // PLL powered off.

      codec_write(0x1B, 0x80); // right justified, 16 bit
      codec_write(0x0B, 0x81); // NDAC is powered and set to 1
      codec_write(0x0C, 0x82); // MDAC is powered and set to 2
      break;

    case 0b0111:
      // Dreamcast mode
      // • 32-bit frame size, 16-bit word size, right-justified
      // • W (WS / LRCLK / FS): 44.1kHz
      // • C (SCK / BCLK): 2.8224MHz (44.1kHz * 2ch * 32 bit frame size)
      // • M (MCLK): 11.2896 (256Fs = 256 * 44.1kHz)

      codec_write(0x04, 0x00); // PLL_CLKIN = MCLK, CODEC_CLKIN = MCLK
      codec_write(0x05, 0x00); // PLL powered off.

      codec_write(0x1B, 0x80); // right justified, 16 bit
      codec_write(0x0B, 0x81); // NDAC is powered and set to 1
      codec_write(0x0C, 0x82); // MDAC is powered and set to 2
      break;

    case 0b1000:
      // PS1 mode
      // • 32-bit frame size, 16-bit word size, right-justified
      // • W (WS / LRCLK / FS): 44.1kHz
      // • C (SCK / BCLK): 2.8224MHz (44.1kHz * 2ch * 32 bit frame size)
      // • M (MCLK): 11.2896 (256Fs = 256 * 44.1kHz)

      codec_write(0x04, 0x00); // PLL_CLKIN = MCLK, CODEC_CLKIN = MCLK
      codec_write(0x05, 0x00); // PLL powered off.

      codec_write(0x1B, 0b10110000); // right justified, 16 bit
      codec_write(0x0B, 0x81); // NDAC is powered and set to 1
      codec_write(0x0C, 0x82); // MDAC is powered and set to 2
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
      // • 24-bit frame size, 16-bit word size, right-justified
      // • W (WS / LRCLK / FS): 55.56kHz
      // • C (SCK / BCLK): 2.667MHz (55.56kHz * 2ch * 24 bit frame size)
      // • no MCLK

      codec_write(0x04, 0b00000111); // PLL_CLKIN = BCLK, CODEC_CLKIN = PLL_CLK
      codec_write(0x05, 0b10010011); // PLL powered up. P = 1, R = 3.
      // J defaults to 4

      codec_write(0x1B, 0x80); // right justified, 16 bit
      codec_write(0x0B, 0x84); // NDAC is powered and set to 4
      codec_write(0x0C, 0x84); // MDAC is powered and set to 4

      break;
      break;

    case 0b1111:
      // ????
      break;

    default:
      // ????
      break;
  }

  // finish up amp init

  if (mode == 0) { // analog mode
    codec_write(0x00, 0x01); // Select page 1
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
      read_pot();

      // mute if value is beyond threshold
      if (INVERT_VOL_POT ? vol_pot < (VOL_POT_MIN + MUTE_THRESHOLD) 
                         : vol_pot > (VOL_POT_MAX - MUTE_THRESHOLD)) {
        mute = true;
        if (headset_connected)
          mute_hp();
        else
          mute_spk();
      } else {
        mute = false;
        if (headset_connected)
          unmute_hp();
        else
          unmute_spk();
        // map ADC value to volume
        // 2. Handle inversion logic
        if (INVERT_VOL_POT) {
          vol = map(vol_pot, VOL_POT_MIN + MUTE_THRESHOLD, VOL_POT_MAX, min_vol, max_vol);
        } else {
          vol = map(vol_pot, VOL_POT_MIN, VOL_POT_MAX - MUTE_THRESHOLD, max_vol, min_vol);
        }

        // handle volume change
        if (!mute) {
          if (headset_connected) {
            hp_vol = vol;
            set_hp_vol(hp_vol);
          } else {
            spk_vol = vol;
            set_spk_vol(spk_vol);
          }
        }
      }
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

    // in analog volume control mode, we want to sample again ASAP
    if (!analog_vol_ctrl) {
      _delay_ms(50);
    }
  }
}