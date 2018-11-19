/*
 *	See https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino for the Arduino driver.
 */

#include <stdint.h>

// https://github.com/adafruit/Adafruit_INA219

#define INA219_ADDRESS (0x40)
#define INA219_REG_CURRENT (0x01)
#define INA219_REG_CALIBRATION (0x05)

WarpStatus read_current(unsigned *milli_amps);
