#include <Adafruit_PCA9685.h>
#include <SparkFunDS1307RTC.h>
#include <SparkFunMicroOLED.h>
#include <SparkFunSX1509.h>
#include <Adafruit_MCP23017.h>
#include "logo.h"
#include <math.h>

const int CURRENT_SENSOR_PIN = A5;

Adafruit_MCP23017 mcp0, mcp1, mcp2, mcp3;

//MicroOLED oled;
MicroOLED oled(MODE_I2C, D7, 0);    // Example I2C declaration RST=D7, DC=LOW (0)
MicroOLED oled2(MODE_I2C, D6, 1);    // Example I2C declaration RST=D7, DC=LOW (0)

//LED Driver
Adafruit_PCA9685 ledDriver = Adafruit_PCA9685(0x40, true);  // Use the default address, but also turn on debugging

// SX1509 I2C address (set by ADDR1 and ADDR0 (00 by default):
const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address
SX1509 io; // Create an SX1509 object to be used throughout

// SX1509 Pins:
const byte SX1509_Switch0 = 0; // Switch for Pump
const byte SX1509_Switch1 = 1; // Switch for ACC1
const byte SX1509_Switch2 = 2; // Switch for ACC2
const byte SX1509_Switch3 = 3; // Switch for ACC3
const byte SX1509_Switch4 = 4; // Switch for ACC3

//Touch Pot Address and Variables
int i2cAddr_0 = 4; // Direct access at i2cAddr, indirect registers at i2cAddr+1
int i2cAddr_1 = 8;
uint8_t prevValue;
uint8_t curValue;
uint8_t prevValue2;
uint8_t curValue2;

//Analog Pins connected to MUXs
const int rlyMuxIn = A2;
const int ledMuxIn = A1;
const int sensorMuxIn = A0;


//variables for calculating Current on ACS712s
float sensorValue = 0;
float voltage;
float current;
const int avgSamples = 150;
double currentWatts;
//int calibratedWattage;


double LED1_A = 0;
double LED2_A = 0;
double R48V_A = 0;
double R12V_A = 0;
double R5V_A = 0;
double PUMP_A = 0;
double system_watts = 0;
double system_amps = 0;


//count variable for timing events
int count = 0;
int displayCounter = 0;
////////////////////////////////////////////////////////////
void setup()
{
  Particle.function("Relay_48V", Relay_48V);
  Particle.function("Relay_PUMP", Relay_PUMP);
  Particle.function("Relay_ACC1", Relay_ACC1);
  Particle.function("Relay_ACC2", Relay_ACC2);
  Particle.function("Relay_ACC3", Relay_ACC3);
  Particle.function("light_1", light_1);
  Particle.function("light_2", light_2);

  Particle.variable("LED1_A", LED1_A);
  Particle.variable("LED2_A", LED2_A);
  Particle.variable("R48V_A", R48V_A);
  Particle.variable("R12V_A", R12V_A);
  Particle.variable("R5V_A", R5V_A);
  Particle.variable("PUMP_A", PUMP_A);
  Particle.variable("system_watts", system_watts);
  Particle.variable("system_amps", system_amps);

  rtc.begin();
  io.begin(SX1509_ADDRESS);


  oled.begin();    // Initialize the OLED
  oled.clear(ALL); // Clear the display's internal memory
  oled.clear(PAGE);
  oled.drawBitmap(openponics);
  oled.display();

  oled2.begin();    // Initialize the OLED
  oled2.clear(ALL); // Clear the display's internal memory
  oled2.clear(PAGE);
  oled2.drawBitmap(openponics);
  oled2.display();
  delay(2000);


  ledDriver.begin();    // This calls Wire.begin()
  ledDriver.setPWMFreq(1000);     // Maximum PWM frequency is 1600

  mcp0.begin();//Growbed Sensor Block
  mcp1.begin(1);//LED Block and 48V Relay
  mcp2.begin(2);//Relay Block
  mcp3.begin(3);//Config Block

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

  //For Tocuh Pot
  prevValue = curValue;
  prevValue2 = curValue2;

  //for current clamp
  //calibratedWattage = getWatts(CURRENT_SENSOR_PIN);

  //for SX1509
  io.pinMode(SX1509_Switch0, INPUT_PULLUP);
  io.pinMode(SX1509_Switch1, INPUT_PULLUP);
  io.pinMode(SX1509_Switch2, INPUT_PULLUP);
  io.pinMode(SX1509_Switch3, INPUT_PULLUP);
  io.pinMode(SX1509_Switch4, INPUT_PULLUP);

  //Initialize Relays all off except 48V
  LED_R48V_enable();
  light_1_OFF();
  light_2_OFF();
  pumpOff();
  ACC1_Relay_Off();
  ACC2_Relay_Off();
  ACC3_Relay_Off();


}
////////////////////////////////////////////////////////////
void loop()
{

//checkIO();
getPowerConsumption();
printOLED1();
//printOLED2();

}
////////////////////////////////////////////////////////////


void checkIO()
{
  if (io.digitalRead(SX1509_Switch0) == LOW)
  {
    pumpOn();
  }
  else
    pumpOff();

  if (io.digitalRead(SX1509_Switch1) == LOW)
  {
    ACC1_Relay_On();
  }
  else
    ACC1_Relay_Off();

  if (io.digitalRead(SX1509_Switch2) == LOW)
  {
    ACC2_Relay_On();
  }
  else
    ACC2_Relay_Off();

  if (io.digitalRead(SX1509_Switch3) == LOW)
  {
    ACC3_Relay_On();
  }
  else
    ACC3_Relay_Off();

  if (io.digitalRead(SX1509_Switch4) == LOW)
  {
    LED_R48V_enable();
  }
  else
    LED_R48V_disable();

  //Control LED 1 with TouchPot
  curValue = ReadTpValue(i2cAddr_0); // faster I2C access than register read
  if (curValue != prevValue)
  {
    for (int led=0; led <= 2; led++)
    {
      ledDriver.setVal(led, map(curValue,0,255,0,4095));
    }
    prevValue = curValue;
  }
  //Control LED2 with TouchPot
  curValue2 = ReadTpValue(i2cAddr_1); // faster I2C access than register read
  if (curValue2 != prevValue2)
  {
    for (int led=3; led <= 5; led++)
    {
      ledDriver.setVal(led, map(curValue2,0,255,0,4095));
    }
    prevValue2 = curValue2;
  }



}
///////////////////////////////////////////////////////////
//Particle.functions to send to Node-Red
int Relay_48V(String cmd) {
  bool relayState = atoi(cmd);
  mcp1.digitalWrite(11, relayState);
}
int Relay_PUMP(String cmd) {
  bool relayState = atoi(cmd);
  mcp2.digitalWrite(11, relayState);
}
int Relay_ACC1(String cmd) {
  bool relayState = atoi(cmd);
  mcp2.digitalWrite(12, relayState);
}
int Relay_ACC2(String cmd) {
  bool relayState = atoi(cmd);
  mcp2.digitalWrite(13, relayState);
}
int Relay_ACC3(String cmd) {
  bool relayState = atoi(cmd);
  mcp2.digitalWrite(14, relayState);
}

int light_1(String cmd)
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

int light_2(String cmd)
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

////////////////////////////////////////////////////////////
void LED_R48V_enable()
{
    mcp1.digitalWrite(11, HIGH);
}
////////////////////////////////////////////////////////////
void LED_R48V_disable()
{
    mcp1.digitalWrite(11, LOW);
}
////////////////////////////////////////////////////////////
void pumpOn()
{
    mcp2.digitalWrite(11, HIGH);
}
////////////////////////////////////////////////////////////
void pumpOff()
{
    mcp2.digitalWrite(11, LOW);
}
////////////////////////////////////////////////////////////
void ACC1_Relay_On()
{
    mcp2.digitalWrite(12, HIGH);
}
////////////////////////////////////////////////////////////
void ACC1_Relay_Off()
{
    mcp2.digitalWrite(12, LOW);
}
////////////////////////////////////////////////////////////
void ACC2_Relay_On()
{
    mcp2.digitalWrite(13, HIGH);
}
////////////////////////////////////////////////////////////
void ACC2_Relay_Off()
{
    mcp2.digitalWrite(13, LOW);
}
////////////////////////////////////////////////////////////
void ACC3_Relay_On()
{
    mcp2.digitalWrite(14, HIGH);
}
////////////////////////////////////////////////////////////
void ACC3_Relay_Off()
{
    mcp2.digitalWrite(14, LOW);
}
////////////////////////////////////////////////////////////
void printOLED1()
{

  if(displayCounter == 100)
  {
  oled.clear(PAGE);

  currentWatts = getWatts(CURRENT_SENSOR_PIN);
  oled.setCursor(0,0);
  oled.print("TOTAL PWR:");
  oled.setCursor(0,10);
  oled.print(currentWatts);
  oled.print("W ");
  oled.setCursor(0,20);
  oled.print(currentWatts/120.0);
  oled.print("A");

    rtc.update();
    oled.setCursor(0,40);
    oled.print(String(rtc.hour()) + ":"); // Print hour
    if (rtc.minute() < 10)
      oled.print('0'); // Print leading '0' for minute
    oled.print(String(rtc.minute()) + ":"); // Print minute
    if (rtc.second() < 10)
      oled.print('0'); // Print leading '0' for second
    oled.print(String(rtc.second())); // Print second

  oled.display();
  displayCounter = 0;
  }
  displayCounter++;

}
////////////////////////////////////////////////////////////
void printOLED2()
{


  //oled2.clear(PAGE);
  //oled2.display();
}
////////////////////////////////////////////////////////////
void getPowerConsumption()
{
  setMuxLED1();
  currentSense(ledMuxIn);//give an Analog Pin to read
  //LED1_V = int(voltage);
  if(current < 0)
  LED1_A = 0;
  else
  LED1_A = current;

  setMuxLED2();
  currentSense(ledMuxIn);//give an Analog Pin to read
  //LED2_V = int(voltage);
  if(current < 0)
  LED2_A = 0;
  else
  LED2_A = current;

  setMuxPump();
  currentSense(rlyMuxIn);
  if(current < 0)
  PUMP_A = 0;
  else
  PUMP_A = current;

  setMux5V();
  currentSense(rlyMuxIn);//give an Analog Pin to read
  //R5V_V = int(voltage);
  if(current < 0)
  R5V_A = 0;
  else
  R5V_A = current;

  setMux12V();
  currentSense(rlyMuxIn);//give an Analog Pin to read
  //R12V_V = int(voltage);
  if(current < 0)
  R12V_A = 0;
  else
  R12V_A = current;

  setMux48V();
  currentSense(rlyMuxIn);//give an Analog Pin to read
  //R48V_V = int(voltage);
  if(current < 0)
  R48V_A = 0;
  else
  R48V_A = current;

  currentWatts = getWatts(CURRENT_SENSOR_PIN);
  system_watts = currentWatts;
  system_amps = currentWatts/120.0;
}
////////////////////////////////////////////////////////////
void currentSense(int pinValue)
{

  sensorValue = analogRead(pinValue);

  // The on-board ADC is 12-bits -> 2^10 = 4096 -> 3.3V / 4096 ~= 0.8mV
  // The voltage is in millivolts
  voltage = sensorValue * 3.3/4096;

  // This will calculate the actual current (in mA)
  // Using the Vref and sensitivity settings you configure

  current = (voltage - 2.5) / 0.100;

/*
  for(int i=0;i<avgSamples;i++)
  {
    sensorValue+=analogRead(pinValue); //read the current from sensor
    delay(2);
  }
  sensorValue=sensorValue/avgSamples;//Taking Average of Samples

  voltage = sensorValue * (3.3/4096.0);

  //current = (voltage - Vref) * sensitivity;

  //((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
  //2.5 is offset(I assumed that arduino is working on 5v so the viout at no current comes
  //out to be 2.5 which is out offset. If your arduino is working on different voltage than
  //you must change the offset according to the input voltage)
  //0.185v(185mV) is rise in output voltage when 1A current flows at input
  //current = (2.5 - voltage )/0.185;
  current = (2.5 - (sensorValue * (3.3 / 4096.0)) )/0.100;
  */
}
////////////////////////////////////////////////////////////
float getWatts(int pin) {

  // RMS voltage
  const double vRMS = 120.0;      // Assumed or measured

  // Parameters for measuring RMS current
  const double offset = 1.65;     // Half the ADC max voltage
  const int numTurns = 2000;      // 1:2000 transformer turns
  const int rBurden = 100;        // Burden resistor value

  int sample;
  double voltage;
  double iPrimary;
  double acc = 0;
  double iRMS;
  double apparentPower;

  // Take a number of samples and calculate RMS current
  for ( int i = 0; i < avgSamples; i++ ) {

      // Read ADC, convert to voltage, remove offset
      sample = analogRead(pin);
      voltage = (sample * 3.3) / 4096;
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
///////////////////////////////////////////////////////////
void light_1_ON()
{
  for (int led=0; led <= 2; led++)
  {
    ledDriver.setVal(led, 4096);
  }
}

void light_1_OFF()
{
  for (int led=0; led <= 2; led++)
  {
        ledDriver.setVal(led, 0);
  }

}
void light_2_ON()
{
  for (int led=3; led <= 5; led++)
  {
    ledDriver.setVal(led, 4096);
  }
}

void light_2_OFF()
{
  for (int led=3; led <= 5; led++)
  {
        ledDriver.setVal(led, 0);
  }

}
//////////////////////////////////////////////////
// Write a Touch Potentiometer register
void WriteTpReg(uint8_t addr, uint8_t data) {
  Wire.beginTransmission(addr+1);
  Wire.write('W');
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}

// Get the Touch Potentiometer value
uint8_t ReadTpValue(uint8_t addr) {
  Wire.requestFrom(addr, 1);
  if (Wire.available()) {
    return Wire.read();
  } else {
    return 0;
  }
}

// Read a Touch Potentiometer register
uint8_t ReadTpReg(uint8_t addr) {
  Wire.beginTransmission(addr+1);
  Wire.write('R');
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(addr+1, 1);
  if (Wire.available()) {
    return Wire.read();
  } else {
    return 0;
  }
}
