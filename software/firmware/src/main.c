#include <stdint.h>
#include <string.h>

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "gpio.h"
#include "i2c.h"

// Initialize the GPIO pins
static void gpio_init()
{
  
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

  // Main loop
  while (1);
}