#include <stdint.h>
#include <string.h>

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "gpio.h"
#include "i2c.h"
#include "i2c/tlv320aic3110.h"

// GPIO pin definitions
static const gpio_t RESET = {&PORTB, 2}; // PB2
// static const gpio_t HPS   = {&PORTA, 7}; // PA7

// Initialize the GPIO pins
static void gpio_init()
{
  // Codec reset, active low
  gpio_output(RESET);
  gpio_set_low(RESET);

  // PORTA.PIN7CTRL |= PORT_PULLUPEN_bm; // enable pullup on PA7

  // ADC on PA7
  ADC0.CTRLA  = 0x07; // enable ADC in free-running 8-bit mode
  ADC0.CTRLC  = 0x01010000; // reduced capacitance mode, set VREF to VDD
  ADC0.MUXPOS = ADC_MUXPOS_AIN6_gc; // select PA6
  // ADC0.COMMAND = 0b1; // start conversion
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

static void codec_init()
{
  // Reset codec
  gpio_set_low(RESET);
  _delay_ms(10);
  gpio_set_high(RESET);
}

int main(void)
{
  // Enable prescaler, and set prescaler division to 4 to run at 5MHz
  CPU_CCP           = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm;

  // Enable interrupts (for I2C target comms and overtemp alerting)
  sei();

  // Initalize the GPIOs
  gpio_init();

  // Initialize as an I2C controller
  i2c_configure(I2C_MODE_STANDARD);

  codec_init(); // reset codec

  // Initialize TLV320AIC3110
  codec_write(0x0, 0x0); //  select page 0
  codec_write(0x01, 0x01); // soft reset
  _delay_ms(10);

  // N64
  // codec_write(0x04, 0b00000111); // PLL_CLKIN = BCLK, CODEC_CLKIN = PLL_CLK
  // codec_write(0x05, 0b10010011); // PLL power up

  // codec_write(0x1B, 0b10000000); //  left-justified, word length 32
  // // codec_write(0x1C, 0x10); //  16 bit offset?
  // codec_write(0x0B, 0x84); // NDAC is powered and set to 1
  // codec_write(0x0C, 0x84); // MDAC is powered and set to 2

  // SNES
  codec_write(0x04, 0x00); // PLL_CLKIN = MCLK, CODEC_CLKIN = MCLK
  codec_write(0x05, 0x00); // PLL powered off.

  codec_write(0x1B, 0x80); //  right justified, 16 bit
  codec_write(0x0B, 0x81); // NDAC is powered and set to 1
  codec_write(0x0C, 0x82); // MDAC is powered and set to 2

  // Wii
  // codec_write(0x04, 0b00000111); // PLL_CLKIN = MCLK, CODEC_CLKIN = MCLK
  // codec_write(0x05, 0x91); // PLL power up
  // codec_write(0x06, 0x20); // PLL power up

  // codec_write(0x1B, 0b10110000); //  left-justified, word length 32
  // codec_write(0x1D, 0b00001000); //  left-justified, word length 32
  // // codec_write(0x1C, 0x10); //  16 bit offset?
  // codec_write(0x0B, 0x84); // NDAC is powered and set to 1
  // codec_write(0x0C, 0x84); // MDAC is powered and set to 2

  // uint8_t osr_buf[3] = {0x0D, 0x00, 0x80}; // 128
  // i2c_write(TLV320_ADDR, osr_buf, 3);

  codec_write(0x74, 0x0); //  DAC -> volume control pin disable
  codec_write(0x44, 0x0); // DAC -> DRC disable
  codec_write(0x41, 0xf0); // DAC -> +24dB gain left
  codec_write(0x42, 0xf0); // DAC -> +24dB gain right

  codec_write(0x0, 0x01); // Select page 1
  codec_write(0x21, 0x4E); // De-pop, power on = 800 ms, step time = 4ms
  // codec_write(0x1F, 0xC4); // HPL and HPR powered up
  codec_write(0x23, 0x44); // LDAC routed to left mixer, RDAC routed to right mixer
  // codec_write(0x2A, 0x1C); // unmute class D left
  // codec_write(0x2B, 0x1C); // unmute class D right
  // codec_write(0x20, 0xC6); // power up class D drivers
  codec_write(0x24, 0x80); // enables HPL output analog volume, set = 0 dB
  codec_write(0x25, 0x80); // enables HPR output analog volume, set = 0 dB
  codec_write(0x26, 0x88); // enables SPL output analog volume, set = 0 dB
  codec_write(0x27, 0x88); // enables SPR output analog volume, set = 0 dB
  codec_write(0x28, 0x06); // enables SPL output analog volume, set = 0 dB
  codec_write(0x29, 0x06); // enables SPR output analog volume, set = 0 dB
  codec_write(0x2E, 0xB); // Micbias set to AVDD

  // no idea if necessary
  codec_write(0x0, 0x0); // select page 0
  codec_write(0x3C, 0x0B); // select DAC DSP Processing Block PRB_P1
  codec_write(0x00, 0x08); // select page 8
  codec_write(0x01, 0x04); // enable adaptive filtering?

  codec_write(0x00, 0x00); // select page 0
  codec_write(0x3F, 0xD6); // power up DAC left and right channels (soft step disable)
  codec_write(0x40, 0x0); // unmute DAC left and right channels

  codec_write(0x43, 0x93); // headset detection enabled

  // Main loop
  while (1) {

    int a6 = 0;
    int a7 = 0;

    // measure PA6
    ADC0.MUXPOS  = ADC_MUXPOS_AIN6_gc; // select PA6
    ADC0.COMMAND = 0b1; // start conversion
    while (ADC0.COMMAND & ADC_STCONV_bm) { _delay_ms(1); }
    a6 = ADC0.RESL;

    // measure PA7
    ADC0.MUXPOS  = ADC_MUXPOS_AIN7_gc; // select PA6
    ADC0.COMMAND = 0b1; // start conversion
    while (ADC0.COMMAND & ADC_STCONV_bm) { _delay_ms(1); }
    a7 = ADC0.RESL;

    // if PA6 == PA7, then headphones are NOT present, so turn them off and enable speakers
    if (a6 > (a7 - 0x0f) && a6 < (a7 + 0x0f)) {
      codec_write(0x0, 0x01); // Select page 1
      codec_write(0x1F, 0x04); // power down headphone drivers
      codec_write(0x2A, 0x1C); // unmute class D left
      codec_write(0x2B, 0x1C); // unmute class D right
      codec_write(0x20, 0xC6); // power up class D drivers
    } else { // PA6 and PA7 are different because tip switch is isolated and pulled high; enable headphones
      codec_write(0x0, 0x01); // Select page 1
      codec_write(0x2A, 0x00); // mute class D left
      codec_write(0x2B, 0x00); // mute class D right
      codec_write(0x20, 0x06); // power down class D drivers
      codec_write(0x1F, 0xC4); // power up headphone drivers
    }

    _delay_ms(100);

    // if (ADC0.RESL > 0xF0) { // if PA7 is floating (high due to pull-up), turn on speakers
    //   codec_write(0x0, 0x01); // Select page 1
    //   codec_write(0x2A, 0x1C); // unmute class D left
    //   codec_write(0x2B, 0x1C); // unmute class D right
    //   codec_write(0x20, 0xC6); // power up class D drivers
    // } else { // if PA7 is low, mute speakers
    //   codec_write(0x0, 0x01); // Select page 1
    //   codec_write(0x2A, 0x00); // mute class D left
    //   codec_write(0x2B, 0x00); // mute class D right
    //   codec_write(0x20, 0x06); // power down class D drivers
    // }
  }
}