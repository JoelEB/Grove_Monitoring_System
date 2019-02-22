#include <Adafruit_PCA9685.h>
#include <SparkFunDS1307RTC.h>
#include <SparkFunMicroOLED.h>
#include <Adafruit_MCP23017.h>
#include "logo.h"
#include <math.h>

//Digital I/O Expander Initialization
Adafruit_MCP23017 mcp0, mcp1, mcp2, mcp3;

//MicroOLED Initialization
MicroOLED oled(MODE_I2C, D7, 0);    // Example I2C declaration RST=D7, DC=LOW (0)

//LED Driver Initialization
Adafruit_PCA9685 ledDriver = Adafruit_PCA9685(0x40, true);  // Use the default address, but also turn on debugging

//Analog Pins connected to MUXs
const int rlyMuxIn = A2;
const int ledMuxIn = A1;
const int sensorMuxIn = A0;

//variables for calculating Current on ACS712s
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
double system_watts = 0;
double system_amps = 0;


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
int displayCounter = 10;

// 24hr timer for resyncing cloud time
unsigned long updateCTime;
////////////////////////////////////////////////////////////
void setup()
{
  Particle.function("Relay_48V", Relay_48V);
  Particle.function("Relay_PUMP", Relay_PUMP);
  Particle.function("Relay_ACC1", Relay_ACC1);
  Particle.function("Relay_ACC2", Relay_ACC2);
  Particle.function("Relay_ACC3", Relay_ACC3);
  Particle.function("LED1analog", LED1analog);
  Particle.function("LED2analog", LED2analog);
  Particle.function("LED1digital", LED1digital);
  Particle.function("LED2digital", LED2digital);

  Particle.variable("LED1_A", LED1_A);
  Particle.variable("LED2_A", LED2_A);
  Particle.variable("R48V_A", R48V_A);
  Particle.variable("R12V_A", R12V_A);
  Particle.variable("R5V_A", R5V_A);
  Particle.variable("PUMP_A", PUMP_A);
  Particle.variable("system_watts", system_watts);
  Particle.variable("system_amps", system_amps);

/*
  Particle.variable("state_48V", state_48V);
  Particle.variable("state_PUMP", state_PUMP);
  Particle.variable("state_ACC1", state_ACC1);
  Particle.variable("state_ACC2", state_ACC2);
  Particle.variable("state_ACC3", state_ACC3);
  Particle.variable("LED1state", LED1state);
  Particle.variable("LED2state", LED2state);
  Particle.variable("LED1value", LED1value);
  Particle.variable("LED2value", LED2value);
*/
  //Initialize onboard real time clock
  rtc.begin();
  //Sync Photon with Internet time and set time zone
  //Time.zone(-6);
  Particle.syncTime();
  updateCTime = millis();

  oled.begin();    // Initialize the OLED
  oled.clear(ALL); // Clear the display's internal memory
  oled.clear(PAGE);
  oled.drawBitmap(openponics);
  oled.display();
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


}
////////////////////////////////////////////////////////////
void loop()
{
  // DST adjstment
  bool daylightSavings = IsDST(Time.day(), Time.month(), Time.weekday());
  Time.zone(daylightSavings? -6 : -7);

  setRelaystate();
  getPowerConsumption();
  if(displayCounter == 10)
  {
    printOLED1();
    displayCounter = 0;
  }
  displayCounter++;



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
void printOLED1()
{

  oled.clear(PAGE);

  //Print current power consumption from current clamp
  //currentWatts = getCurrentPower_Clamp(CURRENT_SENSOR_PIN);
  oled.setCursor(0,0);
  oled.print("TOTAL PWR:");
  oled.setCursor(0,10);
  oled.print(system_watts);
  oled.print("W ");
  oled.setCursor(0,20);
  oled.print(system_amps);
  oled.print("A");

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
  oled.setCursor(0,40);
  oled.print(String(Time.hour()) + "::"); // Print hour
  if (Time.minute() < 10)
    oled.print('0'); // Print leading '0' for minute
  oled.print(String(Time.minute()) + ":"); // Print minute
  if (Time.second() < 10)
    oled.print('0'); // Print leading '0' for second
  oled.print(String(Time.second())); // Print second



  oled.display();

}
////////////////////////////////////////////////////////////
void getPowerConsumption()
{
  setMuxLED1();
  delay(10);
  getCurrent_ACS712(ledMuxIn);//give an Analog Pin to read
  LED1_A = current;

  setMuxLED2();
  delay(10);
  getCurrent_ACS712(ledMuxIn);//give an Analog Pin to read
  LED2_A = current;

  setMuxPump();
  delay(10);
  getCurrent_ACS712(rlyMuxIn);
  //pumpARead = average;
  PUMP_A = current;

  setMux5V();
  delay(10);
  getCurrent_ACS712(rlyMuxIn);//give an Analog Pin to read
  R5V_A = current;

  setMux12V();
  delay(10);
  getCurrent_ACS712(rlyMuxIn);//give an Analog Pin to read
  R12V_A = current;

  setMux48V();
  delay(10);
  getCurrent_ACS712(rlyMuxIn);//give an Analog Pin to read
  R48V_A = current;

  currentWatts = getCurrentPower_Clamp(CURRENT_SENSOR_PIN);
  system_watts = currentWatts;
  system_amps = currentWatts/vRMS;
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
