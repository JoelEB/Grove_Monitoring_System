/******************************************************************************

******************************************************************************/
#include "Particle.h"

#ifndef Atlas_Scientific_H
#define Atlas_Scientific_H

#include "application.h"

/////////////Atlas Scientific Stamp Info//////////////
//#define pH1address 0x63 //99 //default I2C ID number for EZO pH Circuit.
//#define pH2address 0x62 //98

class Atlas {
 public:
  void begin(uint8_t addr);

  ///////Specific Atlas Stamp Functions////////
  //pH STamp


  ////To Do
  //temp compensation
  //switch to uart
  //cal
  //find
  //info
  //change I2C address
  //Plock
  //slope
  //status
  //import/export cal


  ///////Functions that are Global to all Atlas Stamps
  float stamp_reading_single();
  void factoryReset();
  void enableLED();
  void disableLED();
  void tempCompensate(float temp);
  void sleep();

 private:
 //Variables
  uint8_t i2caddr;
  byte error_code = 0;          //used to hold the I2C response code.
  char stamp_data[20];          //we make a 20-byte character array to hold incoming data from the pH circuit.

  void  i2cWrite(char data_command[20]) ;//char array is used becyase command can be a single character or a string of chars
  char* i2cRead();   //char* is used to pass char array back from function to use as needed
};

#endif
