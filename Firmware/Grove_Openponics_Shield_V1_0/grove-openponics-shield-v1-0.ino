//#include <SparkFunDS1307RTC.h>
#include <SparkFunMicroOLED.h>
#include "logo.h"
#include <Adafruit_CCS811.h>
#include <Adafruit_PCA9685.h>
#include <Adafruit_MCP23017.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "Atlas_Scientific.h"
#include <math.h>



///////Atlas-Scientific Stamp Info//////
#define ph1_address 0x63 //Default pH Stamp address = 99 (0x63)
#define ph2_address 0x62
#define temp1_address 0x66  //Default RTD Temperature Stamp address = 102 (0x66)
#define EC1_address 0x64  //Default EC Stamp address = 100 (0x64)

Atlas ph1, ph2, EC1, temp1;//Initialization

double ph1_data, ph2_data, EC1_data, temp1_data = 0;

//pH1_stamp  = V2.1
//pH2_stamp  = V1.9
//EC1_stamp  = V2.13
//RTD1_stamp = V1.3


char computerdata[20];           //we make a 20 byte character array to hold incoming data from a pc/mac/other.
byte received_from_computer = 0; //we need to know how many characters have been received.
byte code = 0;                   //used to hold the I2C response code.
char data_from_stamp[20];                //we make a 20-byte character array to hold incoming data from the pH circuit.
byte in_charc = 0;                //used as a 1 byte buffer to store inbound bytes from the pH Circuit.
byte x = 0;                      //counter used for ph_data array.
int delay_time = 900;                 //used to change the delay needed depending on the command sent to the EZO Class pH Circuit.

/////////////////////BME280 Initialization
#define BME_SCK D4
#define BME_MISO D3
#define BME_MOSI D2
#define BME_CS D5

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

double baro_temp;
double baro_RH;
double pressure;

///////////////CCS81 CO2 Sensor Initialization
Adafruit_CCS811 ccs;

double CO2 = 0;
double TVOC = 0;

///////////////Digital I/O Expander Initialization
Adafruit_MCP23017 mcp0, mcp1, mcp2, mcp3;

///////////////MicroOLED Initialization
MicroOLED oled(MODE_I2C, D6, 1);    // Example I2C declaration RST=D7, DC=LOW (0)

///////////////LED Driver Initialization
Adafruit_PCA9685 ledDriver = Adafruit_PCA9685(0x40, true);  // Use the default address, but also turn on debugging

/////////Analog Pins connected to MUXs
const int rlyMuxIn = A2;
const int ledMuxIn = A1;
const int sensorMuxIn = A0;

//////////////Current Sensing//////////////////
//////variables for calculating Current on ACS712s
const int NUM_SAMPLES = 100;
const int CURRENT_SENSOR_PIN = A5;
int sensorValue = 0;
int average = 0;
double voltage = 0;
double current = 0;

//Variables for calculating Watts and Currnet with AC Clamp
const int avgSamples = 500;
// RMS voltage
const double vRMS = 120.0;      // Assumed or measured
// Parameters for measuring RMS current
const double offset = 1.65;     // Half the ADC max voltage
const int numTurns = 2000;      // 1:2000 transformer turns
const int rBurden = 100;        // Burden resistor value
double currentWatts;

//Current and Wattage Variables to send to Dashboard
double LED1_A = 0;
double LED2_A = 0;
double R48V_A = 0;
double R12V_A = 0;
double R5V_A = 0;
double PUMP_A = 0;

double total_watts = 0;
double total_amps = 0;


//Variables to keep track of Relay and LED states
int state_48V = 1;
int state_PUMP = 0;
int state_ACC1 = 0;
int state_ACC2 = 0;
int state_ACC3 = 0;
int LED1state = 0;
int LED2state = 0;
int LED1value = 0;
int LED2value = 0;

//count variable for timing events
long lastPrint = 0;

// 24hr timer for resyncing cloud time
//unsigned long updateCTime;
STARTUP(WiFi.selectAntenna(ANT_AUTO)); // continually switches at high speed between antennas


bool debug = 1;// set to 0 to turn off Serial debug


////////////////////////////////////////////////////////////
void setup()
{
/*
Up to 15 cloud functions may be registered and each
function name is limited to a maximum of 12 characters
https://docs.particle.io/reference/device-os/firmware/photon/#particle-function
*/
  Particle.function("Relay_48V", Relay_48V);
  Particle.function("Relay_PUMP", Relay_PUMP);
  Particle.function("Relay_ACC1", Relay_ACC1);
  Particle.function("Relay_ACC2", Relay_ACC2);
  Particle.function("Relay_ACC3", Relay_ACC3);
  Particle.function("LED1analog", LED1analog);
  Particle.function("LED2analog", LED2analog);
  Particle.function("LED1digital", LED1digital);
  Particle.function("LED2digital", LED2digital);


/*
Up to 20 cloud variables may be registered and each
variable name is limited to a maximum of 12 characters
*/
  Particle.variable("TVOC", TVOC);
  Particle.variable("CO2", CO2);
  
  Particle.variable("baro_temp", baro_temp);
  Particle.variable("baro_RH", baro_RH);
  Particle.variable("pressure", pressure);
  
  Particle.variable("ph1_data", ph1_data);   
  Particle.variable("ph2_data", ph2_data);
  Particle.variable("EC1_data", EC1_data);
  Particle.variable("temp1_data", temp1_data);
  
  Particle.variable("total_watts", total_watts);
  Particle.variable("total_amps", total_amps);
  
  Particle.variable("R48V_A", R48V_A);
  Particle.variable("R12V_A", R12V_A);

//   Particle.variable("R5V_A", R5V_A);
//   Particle.variable("PUMP_A", PUMP_A);
//   Particle.variable("LED1_A", LED1_A);
//   Particle.variable("LED2_A", LED2_A);


  //Initialize onboard real time clock
  //rtc.begin();
  //Sync Photon with Internet time and set time zone
  //Time.zone(-6);
  //Particle.syncTime();
  //updateCTime = millis();

  oled.begin();    // Initialize the OLED
  oled.clear(ALL); // Clear the display's internal memory
  oled.clear(PAGE);
  oled.drawBitmap(openponics);
  oled.display();
  delay(2000);

  if(debug)
  Serial.begin(9600);           //enable serial port   

  ph1.begin(ph1_address);
  ph1.sleep();
  ph2.begin(ph2_address);
  ph2.sleep();
  temp1.begin(temp1_address);
  temp1.sleep();
  EC1.begin(EC1_address);
  EC1.sleep();
   
  bme.begin(0x77);//enable BME280
   
  ccs.begin(0x5B);//enable CCS811 CO2 Sensor

  ledDriver.begin();              // This calls Wire.begin()
  ledDriver.setPWMFreq(1000);     // Maximum PWM frequency is 1600

  mcp0.begin();//Growbed Sensor Block
  mcp1.begin(1);//LED Block and 48V Relay
  mcp2.begin(2);//Relay Block
  mcp3.begin(3);//Config Block

  //Set all I2C I/O as Outputs and to LOW
  for(int i = 0;i < 16; i++)
  {
    mcp0.pinMode(i, OUTPUT);
    mcp1.pinMode(i, OUTPUT);
    mcp2.pinMode(i, OUTPUT);
    mcp3.pinMode(i, OUTPUT);
  }

  for(int i=0;i<16; i++)
  {
    mcp0.digitalWrite(i, LOW);
    mcp1.digitalWrite(i, LOW);
    mcp2.digitalWrite(i, LOW);
    mcp3.digitalWrite(i, LOW);
  }

  setRelaystate();//Set relays to inital state

}
////////////////////////////////////////////////////////////
void loop()
{
    // This math looks at the current time vs the last time a publish happened
    if(millis() - lastPrint > 15000) //Publishes every 15000 milliseconds, or 15 seconds
    {
        getTotalPower();
        getPowerConsumption();
        get_stamp_data();
        get_air_quality();
        get_BME_data();
        printOLED1();
        // Record when you published
        lastPrint = millis();
    }

  setRelaystate();//set relays every iteration/checks for changes from Node-Red

  if(debug)
  serialDebug();

  // DST adjstment
  //bool daylightSavings = IsDST(Time.day(), Time.month(), Time.weekday());
  //Time.zone(daylightSavings? -6 : -7);
  // Re-sync cloud time every 24 hrs
  /*
  if ((millis() - updateCTime) > (24UL * 60UL * 60UL * 1000UL))
  {
    Particle.syncTime();
    updateCTime = millis();
  }
  */
}
////////////////////////////////////////////////////////////
//////////Particle.functions to send to Node-Red////////////
////////////////////////////////////////////////////////////
int Relay_48V(String cmd) {
  state_48V = atoi(cmd);
  setRelaystate();
}
int Relay_PUMP(String cmd) {
  state_PUMP = atoi(cmd);
  setRelaystate();
}
int Relay_ACC1(String cmd) {
  state_ACC1 = atoi(cmd);
  setRelaystate();
}
int Relay_ACC2(String cmd) {
  state_ACC2 = atoi(cmd);
  setRelaystate();
}
int Relay_ACC3(String cmd) {
  state_ACC3 = atoi(cmd);
  setRelaystate();
}
int LED1digital(String cmd)
{
  bool ledState = atoi(cmd);
  if(ledState == 1)
  {
    for (int led=0; led <= 2; led++)
    {
      ledDriver.setVal(led, 4096);
    }
  }
  else
  {
    for (int led=0; led <= 2; led++)
    {
      ledDriver.setVal(led, 0);
    }
  }
}
int LED2digital(String cmd)
{
  bool ledState = atoi(cmd);
  if(ledState == 1)
  {
    for (int led=3; led <= 5; led++)
    {
      ledDriver.setVal(led, 4096);
    }
  }
  else
  {
    for (int led=3; led <= 5; led++)
    {
      ledDriver.setVal(led, 0);
    }
  }
}
int LED1analog(String cmd)
{
  int curValue = atoi(cmd);
    for (int led=0; led <= 2; led++)
    {
      ledDriver.setVal(led, map(curValue,0,255,0,4095));
    }
}
int LED2analog(String cmd)
{
  int curValue = atoi(cmd);
    for (int led=3; led <= 5; led++)
    {
      ledDriver.setVal(led, map(curValue,0,255,0,4095));
    }
}
///////////////////////////////////////////////////////////
void setRelaystate()
{
  mcp1.digitalWrite(11, state_48V);
  mcp2.digitalWrite(11, state_PUMP);
  mcp2.digitalWrite(12, state_ACC1);
  mcp2.digitalWrite(13, state_ACC2);
  mcp2.digitalWrite(14, state_ACC3);
}
///////////////////////////////////////////////////////////
void setLEDstate()
{
  if(LED1state == 1)
  {
    for (int led=0; led <= 2; led++)
    {
      ledDriver.setVal(led, 4096);
    }
  }
  else
  {
    for (int led=0; led <= 2; led++)
    {
      ledDriver.setVal(led, 0);
    }

  }
  if( LED2state == 1)
  {
    for (int led=3; led <= 5; led++)
    {
      ledDriver.setVal(led, 4096);
    }
  }
  else
  {
    for (int led=3; led <= 5; led++)
    {
      ledDriver.setVal(led, 0);
    }
  }
}
////////////////////////////////////////////////////////////
//Get single readings from all Atlas Scientific Stamps
void get_stamp_data()
{
  temp1_data = temp1.stamp_reading_single(); //get current temp
  temp1.sleep();//sleep
  
  ph1.tempCompensate(float(temp1_data));//compensate with current temp
  ph1_data = ph1.stamp_reading_single();//take single reading
  ph1.sleep();
  
  ph2.tempCompensate(float(temp1_data));
  ph2_data = ph2.stamp_reading_single();
  ph2.sleep();
  
  EC1.tempCompensate(float(temp1_data));
  EC1_data = EC1.stamp_reading_single() / 1000;
  EC1.sleep();
}
float get_temp1_data()
{
  temp1_data = temp1.stamp_reading_single();
  temp1.sleep();
  return float(temp1_data);
}
void get_ph1_data()
{
  ph1.tempCompensate(temp1_data);
  ph1_data = ph1.stamp_reading_single();
  ph1.sleep();
}
void get_ph2_data()
{
  ph2_data = ph2.stamp_reading_single();
  ph2.sleep();
}
void get_EC1_data()
{
  EC1_data = EC1.stamp_reading_single();
  EC1.sleep();
}
////////////////////////////////////////////////

////////////////////////////////////////////////////////////
void getPowerConsumption()
{
//   setMuxLED1();
//   delay(10);
//   getCurrent_ACS712(ledMuxIn);//give an Analog Pin to read
//   LED1_A = current;

//   setMuxLED2();
//   delay(10);
//   getCurrent_ACS712(ledMuxIn);//give an Analog Pin to read
//   LED2_A = current;

//   setMuxPump();
//   delay(10);
//   getCurrent_ACS712(rlyMuxIn);
//   //pumpARead = average;
//   PUMP_A = current;

//   setMux5V();
//   delay(10);
//   getCurrent_ACS712(rlyMuxIn);//give an Analog Pin to read
//   R5V_A = current;

  setMux12V();
  delay(10);
  getCurrent_ACS712(rlyMuxIn);//give an Analog Pin to read
  R12V_A = current;

  setMux48V();
  delay(10);
  getCurrent_ACS712(rlyMuxIn);//give an Analog Pin to read
  R48V_A = current;
}
void getTotalPower()
{
  currentWatts = getCurrentPower_Clamp(CURRENT_SENSOR_PIN);
  total_watts = currentWatts * 0.6;//power factor
  total_amps = currentWatts/vRMS;
}
////////////////////////////////////////////////////////////
void getCurrent_ACS712(int pinValue)
{
  //The on-board ADC is 12-bits -> 2^10 = 4096
  average = 0;
  sensorValue = 0;
  for(int i = 0; i<NUM_SAMPLES; i++)
  {
    sensorValue += analogRead(pinValue); //read the current from sensor
    //delay(2);
  }
  average = sensorValue / NUM_SAMPLES;
  voltage = (average * 3.3)/4095;


  //sensorValue = analogRead(pinValue);
  //voltage = (sensorValue * 3.3)/4095;
  current = abs((voltage - 2.51) / 0.100);

}
////////////////////////////////////////////////////////////
float getCurrentPower_Clamp(int pin)
{
  int sample = 0;
  double voltage = 0;
  double iPrimary = 0;
  double acc = 0;
  double iRMS = 0;
  double apparentPower = 0;

  // Take a number of samples and calculate RMS current
  for ( int i = 0; i < avgSamples; i++ ) {

      // Read ADC, convert to voltage, remove offset
      sample = analogRead(pin);
      voltage = (sample * 3.3) / 4095;
      voltage = voltage - offset;

      // Calculate the sensed current
      iPrimary = (voltage / rBurden) * numTurns;

      // Square current and add to accumulator
      acc += pow(iPrimary, 2);
  }

  // Calculate RMS from accumulated values
  iRMS = sqrt(acc / avgSamples);

  // Calculate apparent power and return it
  apparentPower = vRMS * iRMS;
  return apparentPower;
}
/////////////////////////////////////////////////////////////
bool IsDST(int dayOfMonth, int month, int dayOfWeek)
{
  if (month < 3 || month > 11) {
    return false;
  }
  if (month > 3 && month < 11) {
    return true;
  }

  int previousSunday = dayOfMonth - (dayOfWeek - 1);
  if (month == 3)
  {
    return previousSunday >= 8;
  }
  return previousSunday <= 0;
}
////////////////////////////////////////////////////////////
//The following Functions Set the S0,S1, and S2 pins on the
//corresponding MUXs to analogRead the Isense pins connected to each
////////////////////////////////////////////////////////////
void setMuxLED1()
{
    mcp1.digitalWrite(13, LOW);//S0
    mcp1.digitalWrite(14, LOW);//S1
    mcp1.digitalWrite(15, LOW);//S2
    //set LEDMUX to Y0 output
}
////////////////////////////////////////////////////////////
void setMuxLED2()
{
    mcp1.digitalWrite(13, HIGH);
    mcp1.digitalWrite(14, LOW);
    mcp1.digitalWrite(15, LOW);
    //set LEDMUX to Y1 output
}
////////////////////////////////////////////////////////////
void setMuxLED3()
{
    mcp1.digitalWrite(13, LOW);
    mcp1.digitalWrite(14, HIGH);
    mcp1.digitalWrite(15, LOW);
    //set LEDMUX to Y2 output
}
////////////////////////////////////////////////////////////
void setMux48V()
{
    mcp2.digitalWrite(8, LOW);//S0
    mcp2.digitalWrite(9, LOW);//S1
    mcp2.digitalWrite(10, HIGH);//S2
    //set RLYMUX to Y4 output
}
////////////////////////////////////////////////////////////
void setMux12V()
{
    mcp2.digitalWrite(8, HIGH);
    mcp2.digitalWrite(9, HIGH);
    mcp2.digitalWrite(10, HIGH);
    //set RLYMUX to Y7 output
}
////////////////////////////////////////////////////////////
void setMux5V()
{
    mcp2.digitalWrite(8, LOW);
    mcp2.digitalWrite(9, HIGH);
    mcp2.digitalWrite(10, HIGH);
    //set RLYMUX to Y6 output
}
////////////////////////////////////////////////////////////
void setMuxPump()
{
    mcp2.digitalWrite(8, HIGH);
    mcp2.digitalWrite(9, LOW);
    mcp2.digitalWrite(10, HIGH);
    //set RLYMUX to Y5 output
}


void serialDebug() 
{                                                             

  if (Serial.available() > 0)                                            //if data is holding in the serial buffer
  {
    received_from_computer = Serial.readBytesUntil(13, computerdata, 20); //we read the data sent from the serial monitor(pc/mac/other) until we see a <CR>. We also count how many characters have been received.
    computerdata[received_from_computer] = 0;                             //stop the buffer from transmitting leftovers or garbage.
    computerdata[0] = tolower(computerdata[0]);                           //we make sure the first char in the string is lower case.
    if (computerdata[0] == 'c' || computerdata[0] == 'r')
     delay_time = 900;     //if a command has been sent to calibrate or take a reading we wait 1800ms so that the circuit has time to take the reading.
    else delay_time = 300;                                                     //if any other command has been sent we wait only 300ms.


    Wire.beginTransmission(EC1_address); //call the circuit by its ID number.
    Wire.write(computerdata);        //transmit the command that was sent through the serial port.
    Wire.endTransmission();          //end the I2C data transmission.

    if (strcmp(computerdata, "sleep") != 0)  //if the command that has been sent is NOT the sleep command, wait the correct amount of time and request data.
    {                                            //if it is the sleep command, we do nothing. Issuing a sleep command and then requesting data will wake the pH circuit.



    delay(delay_time);                     //wait the correct amount of time for the circuit to complete its instruction.

    Wire.requestFrom(EC1_address, 20, 1); //call the circuit and request 20 bytes (this may be more than we need)
    code = Wire.read();               //the first byte is the response code, we read this separately.

    switch (code) {                  //switch case based on what the response code is.
      case 1:                        //decimal 1.
        Serial.println("Success");   //means the command was successful.
        break;                       //exits the switch case.

      case 2:                        //decimal 2.
        Serial.println("Failed");    //means the command has failed.
        break;                       //exits the switch case.

      case 254:                      //decimal 254.
        Serial.println("Pending");   //means the command has not yet been finished calculating.
        break;                       //exits the switch case.

      case 255:                      //decimal 255.
        Serial.println("No Data");   //means there is no further data to send.
        break;                       //exits the switch case.
    }





    while (Wire.available()) 
    {                                  //are there bytes to receive.
      in_charc = Wire.read();           //receive a byte.
      data_from_stamp[x] = in_charc;            //load this byte into our array.
      x += 1;                          //incur the counter for the array element.
      if (in_charc == 0) 
      {              //if we see that we have been sent a null command.
        x = 0;                         //reset the counter i to 0.
        Wire.endTransmission();        //end the I2C data transmission.
        break;                         //exit the while loop.
      }
    }

    Serial.println(data_from_stamp);          //print the data.
  }
 }
}
////////////////////////////////////////////
void get_air_quality()
{
  //calibrate temperature sensor
  while(!ccs.available());
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);

  if(ccs.available())
  {
    float temp = ccs.calculateTemperature();
    if(!ccs.readData())
    {
      CO2 = ccs.geteCO2();
      TVOC = ccs.getTVOC();
    }
  }
}
////////////////////////////////////////////
void get_BME_data()
{
    baro_temp = bme.readTemperature();

    pressure = bme.readPressure();

    baro_RH = bme.readHumidity();
}
////////////////////////////////////////////////////////////
void printOLED1()
{

  oled.clear(PAGE);
  oled.setCursor(0,0);
  oled.print("pH1:");
  oled.print(ph1_data);
 
  oled.setCursor(0,12);
  oled.print("pH2:");
  oled.print(ph2_data);
  
  oled.setCursor(0,24);
  oled.print("EC:");
  oled.print(EC1_data);
 
  oled.setCursor(0,36);
  oled.print("T:");
  oled.print((temp1_data * 9/5) + 32);
  oled.print("*F");
//   oled.setCursor(0,0);
//   oled.print("TOTAL PWR:");
//   oled.setCursor(0,10);
//   oled.print(total_watts);
//   oled.print("W ");
//   oled.setCursor(0,20);
//   oled.print(total_amps);
//   oled.print("A");

  //Print current time from RTC
  /*
  rtc.update();
  oled.setCursor(0,40);
  oled.print(String(rtc.hour()) + ":"); // Print hour
  if (rtc.minute() < 10)
    oled.print('0'); // Print leading '0' for minute
  oled.print(String(rtc.minute()) + ":"); // Print minute
  if (rtc.second() < 10)
    oled.print('0'); // Print leading '0' for second
  oled.print(String(rtc.second())); // Print second
  */

  //Print current time from Internet
//   oled.setCursor(0,40);
//   oled.print(String(Time.hour()) + "::"); // Print hour
//   if (Time.minute() < 10)
//     oled.print('0'); // Print leading '0' for minute
//   oled.print(String(Time.minute()) + ":"); // Print minute
//   if (Time.second() < 10)
//     oled.print('0'); // Print leading '0' for second
//   oled.print(String(Time.second())); // Print second



  oled.display();

}
////////////////////////////////////////////////////////////



