#include "Atlas_Scientific.h"

void Atlas::begin(uint8_t addr)
{
    i2caddr = addr;
 	Wire.begin();
}

void Atlas::factoryReset()
{
  i2cWrite("Factory");
}

void Atlas::enableLED()
{
  i2cWrite("L,1");
}

void Atlas::disableLED()
{
  i2cWrite("L,0");
}

float Atlas::stamp_reading_single()
{
  float char_array;

  i2cWrite("r");
  char_array = atof(i2cRead());
  return char_array;
}

void Atlas::sleep()
{
  i2cWrite("Sleep");
}


void Atlas::tempCompensate(float tempC)
{
  // Use String to convert the float to a string - theirs probably a better way but this is easy.
  String temperatureString = "T," + String(tempC);
  
  // You might be able to fix this rather than allocate a new char[] every time.
  int length = temperatureString.length();
  char tempC_array[length];
  	  
  // Convert the string to char[]
  temperatureString.toCharArray(tempC_array, length + 1);
  
  i2cWrite(tempC_array); 	  
  
}
//////////pH Stamp//////////////////


///////////////////////////////////////////////////////////////////////////////
// Private Methods
///////////////////////////////////////////////////////////////////////////////

void Atlas::i2cWrite(char data_command[20])
{
  int time_;

  if (data_command[0] == 'r')
    time_ = 900;     //if a command has been sent to calibrate or take a reading we wait 1800ms so that the circuit has time to take the reading.
  else
    time_ = 300;
                                                     //if any other command has been sent we wait only 300ms.
  Wire.beginTransmission(i2caddr);
  Wire.write(data_command);
  Wire.endTransmission();
  delay(time_);                     //wait the correct amount of time for the circuit to complete its instruction.
}


char * Atlas::i2cRead()
{
  byte in_char = 0; //used as a 1 byte buffer to store inbound bytes from the pH Circuit.
  byte i = 0;       //counter used for ph_data array.

  Wire.requestFrom(i2caddr, 20, 1); //call the circuit and request 20 bytes (this may be more than we need)

  error_code = Wire.read();               //the first byte is the response code, we read this separately.

  while (Wire.available())
  {         //are there bytes to receive.
    in_char = Wire.read();           //receive a byte.
    stamp_data[i] = in_char;            //load this byte into our array.
    i += 1;                          //incur the counter for the array element.
  if (in_char == 0)
  {              //if we see that we have been sent a null command.
    i = 0;                         //reset the counter i to 0.
    Wire.endTransmission();        //end the I2C data transmission.
    break;                         //exit the while loop.
  }
 }
  return stamp_data;

}
