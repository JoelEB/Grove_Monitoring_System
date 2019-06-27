/*************************************************** 
  This is a library for our Adafruit 16-channel PWM & Servo driver

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
  
  Adapted for Spark Core by Paul Kourany, Sept. 15, 2014
  Adapted for Grove Labs by Chad Bean, Nov. 13, 2014
  Adapted for Grove Labs by Louis DeScioli, Dec 12, 2014
 ****************************************************/

#ifndef _Adafruit_PCA9685_H
#define _Adafruit_PCA9685_H

#include "application.h"

// Register addresses
#define PCA9685_MODE1 0x00
// #define PCA9685_MODE2 0x01
// #define PCA9685_SUBADR1 0x02
// #define PCA9685_SUBADR2 0x03
// #define PCA9685_SUBADR3 0x04
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_LED0_ON_H 0x07
#define PCA9685_LED0_OFF_L 0x08
#define PCA9685_LED0_OFF_H 0x09
// #define PCA9685_ALLLED_ON_L 0xFA
// #define PCA9685_ALLLED_ON_H 0xFB
// #define PCA9685_ALLLED_OFF_L 0xFC
// #define PCA9685_ALLLED_OFF_H 0xFD
#define PCA9685_PRESCALE 0xFE


class Adafruit_PCA9685 {
 public:
  Adafruit_PCA9685(uint8_t addr = 0x40, bool debug = false);
  void begin(void);
  void reset(void);
  void setPWMFreq(float freq);
  void setPWM(uint8_t ledNum, uint16_t on, uint16_t off);
  void setVal(uint8_t ledNum, uint16_t val, bool invert=false);
  uint16_t readPWMOff(uint8_t ledNum);
  uint16_t readPWMOn(uint8_t ledNum);

 private:
  uint8_t i2caddr;
  bool debugging;

  uint8_t read8(uint8_t addr);
  void write8(uint8_t addr, uint8_t val);
};

#endif
