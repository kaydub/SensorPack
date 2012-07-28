/*************************************************************************
    VoltBarn Sensor Pack v2.0
    Ken Wallich  
    2009-2012 VoltBarn
    
    Thanks to all the amazing people whose code made this so much easier...
    
    SHT15 sensor based on code from:
    Original code from the tutorial at: http://www.glacialwanderer.com/hobbyrobotics/?p=5
    Then incorporated the sample code from Sensirion
       
    BMP085 Extended Example Code
    by: Jim Lindblom
    SparkFun Electronics
    date: 1/18/11
    license: CC BY-SA v3.0 - http://creativecommons.org/licenses/by-sa/3.0/
   
    Arduino timer based on code from:
    KHM 2008 / Lab3/  Martin Nawrath nawrath@khm.de
    Kunsthochschule fuer Medien Koeln
    Academy of Media Arts Cologne
    
    Wind sensor code based on:
    Sparkfun forums for the Weather Station: http://www.sparkfun.com/commerce/product_info.php?products_id=8942
    from Steve Austin's code, modified the border resistance values based on my experimentation, and changed
    the assumed orientation of the weather station to have the wind vane point north, and added rain sensor
    calculations.
    
    This is fairly challenging to get working while conserving battery power. You don't want to miss rain or wind events,
    but that requires having interrupts enabled. That means that the Arduino won't go to sleep if there's even a small
    breeze. The other challenge is that when asleep, the timers stop, so you can't measure passed time with millis().
    
    That would result in highly inaccurate wind speed calculations, so when a wind event comes in, we turn off sleep, and
    set the radio transmission timer to a shorter interval, resetting after we've sent a packet. 
    
    We still only turn on the XBee radio periodically, so you don't get instantanous wind or rain readings,
    but that will help conserve battery life.
*************************************************************************/
char *SensorPackID = {"yourIDHere"};
char *protocolVersion = "1.0"; // Version of the packet stream, in case we introduce an incompatibility later
//#define WINDRAIN true  // Comment this out if you aren't attaching the wind/rain sensors
//#define WIREDTEST // define this if using serial port for debugging, and not using an XBee

/* various pin definitions */

#define ledPin 5
#define sleepPin 4 // To power down XBee

#define timeoutInterval 500 // This is around 10 minutes, we try to calculate this as seconds, while asleep, time stops
unsigned long nintMillis = 8000; // Set to the number of millis for the watchdog timeout in setup, default to 8000
#ifdef WINDRAIN
#define windTimeoutInterval 120 // If we're getting wind, how long should the timeout be, in milliseconds?
#define rainTimeoutInterval 120 // If it's raining, the timeout interval, in milliseconds
#endif //WINDRAIN
long int timeout = millis() + timeoutInterval;

/* Arduino timer support */
#include <avr/sleep.h>
#include <avr/wdt.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
int nint = 0;
volatile boolean f_wdt=1;


/* Definitions for SHT15 temperature/humidity sensor */
int temperatureCommand  = B00000011;  // command used to read temperature
int humidityCommand = B00000101;  // command used to read humidity

int clockPin = 6;  // pin used for clock - SCK
int dataPin  = 7;  // pin used for data - DATA
int ack;  // track acknowledgment for errors
int val;  
float temperature;          
float humidity;
float hg;

/* Definitions for SCP1000 barametric pressure sensor */
// define spi bus pins (these are defined by the SPI library)


char rev_in_byte;	    
int temp_in;
float temp_f;
unsigned long pressure_lsb;
unsigned long pressure_msb;
unsigned long temp_pressure;
unsigned long pressure;

/* End of SCP1000 definitions */

/* Definitons for BMP085 */

#include <Wire.h>

#define BMP085_ADDRESS 0x77  // I2C address of BMP085

const unsigned char OSS = 0;  // Oversampling Setting

// Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 

short bmp_temperature;
long bmp_pressure;

/* End of BMP085 definitions */
#include <Wire.h>   // include Wire library for I2C

// Definitions for wind/rain sensor from SparkFun */
// Initial code thanks to Steve (Austin?) from the SparkFun forums

#ifdef WINDRAIN

#define PIN_ANEMOMETER  2     // Digital 2 - for exernal interrupt 2
#define PIN_RAINBUCKET  3     // Digital 3 - for external interrupt 1
#define PIN_VANE        1     // Analog 1  - For SensorPack boards, this will be A7, since they work with the Arduino Fio.

// How often we want to calculate wind speed or direction
#define MSECS_CALC_WIND_SPEED 5000
#define MSECS_CALC_WIND_DIR   5000

volatile int numRevsAnemometer = 0; // Incremented in the interrupt
unsigned long nextCalcSpeed = 0;                // When we next calc the wind speed
unsigned long nextCalcDir = 0;                  // When we next calc the direction
unsigned long time;                         // Millis() at each start of loop().
boolean windEvent = false;
boolean rainEvent = false;
float rainfall = 0;
float totalRainfall = 0;
float windSpeed = 0;
float windGust = 0;
unsigned int windDirection = 0;
  
#define NUMDIRS 8
// adc values are based on 5V input. Since this may run off a variable input batteries, we compensate for this in calcWindDir
unsigned long   adc[NUMDIRS] = {26, 50, 77, 118, 161, 200, 222, 256}; 
unsigned long   cdc[NUMDIRS] = {180, 225, 270, 135, 315, 90, 45, 0}; // Pointing north
char *strVals[NUMDIRS] = {"S","SW","W","SE","NW","E","NE","N"};

unsigned long numBuckettip = 0;

/* 

Here's the original code for adc/cdc I used for reference. I was assuming the unit is pointing west. 
Added my tweeks since I wanted mine pointing north, and some alternate code for 16 directions, 
also pointing north. This might be handy for interpolation, if you want it facing a different direction

Sector   Reading  Direction
  0        18        W
  1        33        NW
  2        57        N
  7        97        SW
  3       139        NE
  6       183        S
  5       208        SE
  4       232        E
  
  0  18  N
  1  33  NE
  2  57  E
  3  139  SE
  4  232  S
  5  208  SW
  6  183  W
  7  97  NW
  
  0  18  N
  1  33  NE
  2  57  E
  7  97  NW
  3  139  SE
  6  183  W
  5  208  SW
  4  232  S
  

#define NUMDIRS 8
unsigned long   adc[NUMDIRS] = {26, 45, 77, 118, 161, 196, 220, 256};
//unsigned long   cdc[NUMDIRS] = {270, 315, 0, 225, 45, 180, 135, 90}; // Pointing west
unsigned long   cdc[NUMDIRS] = {0, 45, 90, 315, 135, 270, 225, 180}; // Pointing north

// These directions match 1-for-1 with the values in adc, but
// will have to be adjusted as noted above. Modify 'dirOffset'
// to which direction is 'away' (it's West here).
// char *strVals[NUMDIRS] = {"W","NW","N","SW","NE","S","SE","E"};
// Reorient default direction to point north. 
char *strVals[NUMDIRS] = {"N","NE","E","NW","SE","W","SW","S"}; 
byte dirOffset=0;

*/


/* End of wind/rain sensor definitions */

#endif //WINDRAIN

//#define FAN_ASPIRATED
#define fanPin 8 // If you're making a fan-asperated enclosure, you can control it here. It'll turn the fan on if the temp is > 90

// Note that an Arduino Fio has 8 analog pins, instead of 6 as on many other Arduinos. So, if using a Fio, 
// pins 20, 21 can also be made unused, depending on whether you're using the wind/rain sensor
// Also note that analog to digital pin mapping A0 = D14, A1 = D15, etc.
int unusedPins[] = {9, 10, 11, 12, 13, 16, 19}; // We'll set these to output to save a little power

void setup() {
  byte clr;
  
  // Setup pins we're using, and not using
  
  pinMode(ledPin, OUTPUT);
  
  blink(3); // Visual indicator of starting setup
    
  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, HIGH); // Using pin 8 didn't get us > 3V, which we need to start the fan, using it to pull down

  pinMode(sleepPin, OUTPUT);
  
  // Set unused pins to OUTPUT, and turn 'em down
  for (int i = 0; i < sizeof(unusedPins); i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
  
 #ifdef WINDRAIN
  // Wind/Rain sensor variable setup, and attach interrupts for the windspeed and rainbucket
   pinMode(PIN_ANEMOMETER, INPUT);
   digitalWrite(PIN_ANEMOMETER, HIGH);
   pinMode(PIN_RAINBUCKET, INPUT);
   digitalWrite(PIN_RAINBUCKET, HIGH);
   attachInterrupt(0, countAnemometer, FALLING);
   attachInterrupt(1, countRainmeter, FALLING);
   nextCalcSpeed = millis() + MSECS_CALC_WIND_SPEED;
   nextCalcDir   = millis() + MSECS_CALC_WIND_DIR;
 #endif
    
  // Set XBee up to send all serial data out
  Serial.begin(9600);
 
  // Set up Xbee
  digitalWrite(sleepPin, LOW); // Wake XBee up, in case it was asleep and we had a reset
  delay(20); // Pause so XBee can wake up; Specs say wake up time is around 13ms
  Serial.print("\rVoltBarn SensorPack 4.0 - ");
  Serial.print(SensorPackID);
  Serial.println(" starting up\r");
#ifndef WIREDTEST
  initXBee();
#endif //WIREDTEST
  
  /* Set up Arduino sleep watchdog */
  // CPU Sleep Modes 
  // SM2 SM1 SM0 Sleep Mode
  // 0    0  0 Idle
  // 0    0  1 ADC Noise Reduction
  // 0    1  0 Power-down
  // 0    1  1 Power-save
  // 1    0  0 Reserved
  // 1    0  1 Reserved
  // 1    1  0 Standby(1)

  cbi( SMCR,SE );      // sleep enable, power down mode
  cbi( SMCR,SM0 );     // power down mode
  sbi( SMCR,SM1 );     // power down mode
  cbi( SMCR,SM2 );     // power down mode

  // 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
  // 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
  setup_watchdog(9);
  nintMillis = 8000; // Set to number of milliseconds of watchdog timer
 
 #ifdef WIREDTEST
   Serial.println("Initializing pressure");
 #endif
  Wire.begin();
  bmp085Calibration();
  blink(5);
 #ifdef WIREDTEST
   Serial.println("Pressure initialized");
 #endif
}


void loop () {
 #ifdef WIREDTEST
   Serial.println("Start of loop");
   sendReading();
 #endif
#ifdef WINDRAIN
   time = millis();
   
   if (windEvent == true) { // Only do stuff if there was an interrupt at some point
     if (time >= nextCalcSpeed) {
        calcWindSpeed();
        nextCalcSpeed = time + MSECS_CALC_WIND_SPEED;
     }
     if (time >= nextCalcDir) {
        calcWindDir();
        nextCalcDir = time + MSECS_CALC_WIND_DIR;
     }
     windEvent = false;
     //sendReading(); //For debugging every time there's a wind event
   }
 
   if (numBuckettip > 0) { // We had rain events
     rainfall = numBuckettip * 0.011; // This is how much rain we got since we last reported
     totalRainfall += rainfall; // Keep a cumulative counter, so if packets are lost, we don't loose track
     numBuckettip = 0; // reset bucket tip count
   }
   
  if (windEvent && (nint > (windTimeoutInterval / (nintMillis / 1000)))) {
    sendReading();
    nint = 0;
  } else if (rainEvent && (nint > (rainTimeoutInterval / (nintMillis / 1000)))) {
    sendReading();
    nint = 0;
  } else
 #endif //WINDRAIN
 
  // nint is incremented after every sleep cycle, 8 seconds per. Only send data every timeoutInterval seconds
  if (nint > (timeoutInterval / (nintMillis / 1000))) {
    sendReading();
    nint = 0;
    timeout = millis() + timeoutInterval;
  } // End of routines that will send data
  
  digitalWrite(sleepPin, HIGH); // Put XBee to sleep
  
#ifdef FAN_ASPIRATED
  // Turn the fan on or off
  if (temp_f > 90) 
    digitalWrite(fanPin, LOW); 
  else
    digitalWrite(fanPin, HIGH);
#endif FAN_ASPIRATED
    
  /* Arduino sleep */
  if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
    f_wdt=0;       // reset flag

    nint++;
    blink(2); // Indicate we did a loop with output
    // If we had a wind interrupt, don't put the Arduino to sleep until we've sent a packet
#ifdef WINDRAIN
    if (!windEvent)
      system_sleep();
  }  
#else
    system_sleep();
  }
#endif //WINDRAIN
}


/* Setup for XBee, put in data mode */
boolean initXBee() {
  Serial.print("Initializing XBee\r");
  delay(1100);
  Serial.print("+++");
  delay(1100);

  char thisByte = 0;
  while (thisByte != '\r') {
    if (Serial.available() > 0) {
      thisByte = Serial.read();
    }
  }  
    
    
    // To prevent a deadlock, if we don't get a response, fail
  //    if (millis() > timeout)
  //      return(false);

  // Put the XBee in data mode, sleep mode 1 (pin hibernate, using sleepPin);

  //Serial.print("ATRO2\r"); // Set Xbee retries to 2
  Serial.print("ATSM1\r"); // Set Sleep Mode 1
  Serial.print("ATWR\r"); // Write to non-volatile memory
  //Serial.print("ATRO0\r"); // Send characters as they're written to the serial port, don't buffer
  Serial.print("ATCN\r"); // Exit Command Mode
  return(true);
}

void blink(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
  }
}

void sendReading() {
  int ih;
  int it;
  
    digitalWrite(sleepPin, LOW); // Wake XBee up and send current sensor data
    Serial.begin(9600); // re-enable serial, trying to eliminate the initial garbage byte
    delay(30); // Pause so XBee can wake up
    
    getTempHumidity();
    getTempPressure();
   
    // SensorID
    Serial.print(":id:");
    Serial.print(SensorPackID); // Send the ID of this SensorPack.
    
    Serial.print(":v:");
    Serial.print(protocolVersion); // Send the version of the communications protocol
    
    // Temperatures
    Serial.print(":T:");
    it = int(temperature * 100);
    Serial.print(it/100);
    Serial.print(".");
    Serial.print(it%100);
    //Serial.print(temperature, DEC);
    Serial.print(":t:");
    it = int(temp_f * 100);
    Serial.print(it/100);
    Serial.print(".");
    Serial.print(it%100);
    
    // Humidity
    Serial.print(":h:");
    ih = int(humidity * 100);
    Serial.print(ih/100);
    Serial.print(".");
    Serial.print(ih%100);
    //Serial.print(humidity, DEC);
    
    // Barometric pressure
    Serial.print(":p:");
    Serial.print(hg);
    
 #ifdef WINDRAIN
    //calcWindSpeed(); Wind speed calculated based on a 5 second timeout, don't want to recalculate here
    calcWindDir();
    Serial.print(":w:");
    Serial.print(windSpeed);
    Serial.print(":g:");
    Serial.print(windGust);
    Serial.print(":d:");
    Serial.print(windDirection);
    Serial.print(":r:");
    Serial.print(rainfall);
    Serial.print(":R:");
    Serial.print(totalRainfall);
    // reset various win/rain counters
    windSpeed = 0;
    windGust = 0;
    rainfall = 0;
    windEvent = false;
    rainEvent = false;
 #endif //WINDRAIN
    Serial.print(':');
    Serial.print('\r');
    delay(10); // So output buffer will clear
 #ifdef WIREDTEST
    Serial.println(' ');
 #endif
}

void getTempHumidity() {
  
  /* Constants for calcs */
  float C1=-4.0; // for 12 bit
  float C2= 0.0405; // for 12 bit
  float C3=-0.0000028; // for 12 bit
  float T1=0.01; // for 14 bit @5V
  float T2=0.00008; // for 14 bit @5V
  
  float rh; 
  float t; 
  float rh_lin; 
  float rh_true;
  float t_C;
  
  /* Original sample code
  const float C1=-4.0; // for 12 bit
  const float C2= 0.0405; // for 12 bit
  const float C3=-0.0000028; // for 12 bit
  const float T1=0.01; // for 14 bit @5V
  const float T2=0.00008; for 14 bit @5V

  float rh=*p_humidity; 
  float t=*p_temperature; 
  float rh_lin; 
  float rh_true;
  float t_C;
  
  t_C=t*0.01 – 40; //calc. Temperature from ticks to [␣C]
  rh_lin=C3*rh*rh + C2*rh + C1;  //calc. Humidity from ticks to [%RH]
  rh_true=(t_C-25)*(T1+T2*rh)+rh_lin; //calc. Temperature compensated humidity [%RH]
  
  if(rh_true>100)rh_true=100; //cut if the value is outside of
  if(rh_true<0.1)rh_true=0.1;  //the physical possible range

  *p_temperature=t_C; 
  *p_humidity=rh_true; //return humidity[%RH]	
*/

  // read the temperature and convert it to F
  sendCommandSHT(temperatureCommand, dataPin, clockPin);
  waitForResultSHT(dataPin);
  val = getData16SHT(dataPin, clockPin);
  t_C = (float)val * 0.01 - 40; //centigrade
  temperature = -40.0 + 0.018 * (float)val; // farienheit
  skipCrcSHT(dataPin, clockPin);
  
  // read the humidity
  sendCommandSHT(humidityCommand, dataPin, clockPin);
  waitForResultSHT(dataPin);
  val = getData16SHT(dataPin, clockPin);
  skipCrcSHT(dataPin, clockPin);
  rh = (float)val;
  rh_lin=C3*rh*rh + C2*rh + C1;  //calc. Humidity from ticks to [%RH]
  rh_true=(t_C-25)*(T1+T2*rh)+rh_lin; //calc. Temperature compensated humidity [%RH]
  
  if(rh_true>100)rh_true=100; //cut if the value is outside of
  if(rh_true<0.1)rh_true=0.1;  //the physical possible range
  humidity = min(rh_true, 100.0); //Above fixes still allow readings of 100.x -- kmw
 
  
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

void getTempPressure() {
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

// Temperature  
  unsigned int ut = bmp085ReadUT();

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;
  
  short temp_in;

  temp_in =  ((b5 + 8)>>4);  
  temp_f = ((1.8) * (temp_in / 10.0)) + 32.0;

// Pressure  
  unsigned int up = bmp085ReadUP();
    
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
//  p /= 4;
  hg = p * .0002954;
}


// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.receive();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.receive();
  lsb = Wire.receive();
  
  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(0xF4);
  Wire.send(0x2E);
  Wire.endTransmission();
  
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(0xF4);
  Wire.send(0x34 + (OSS<<6));
  Wire.endTransmission();
  
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.receive();
  lsb = Wire.receive();
  xlsb = Wire.receive();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}

// END of MPG0085 routines

// commands for reading/sending data to a SHTx sensor

int shiftIn(int dataPin, int clockPin, int numBits) {
  int ret = 0;

  for (int i=0; i<numBits; ++i) {
    digitalWrite(clockPin, HIGH);
    //delay(10); not needed :)
    ret = ret*2 + digitalRead(dataPin);
    digitalWrite(clockPin, LOW);
  }
  return(ret);
}
// send a command to the SHTx sensor
void sendCommandSHT(int command, int dataPin, int clockPin) {
  int ack;

  // transmission start
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, LOW);
  
  // shift out the command (the 3 MSB are address and must be 000, the last 5 bits are the command)
  shiftOut(dataPin, clockPin, MSBFIRST, command);
  
  /* Too much noise transmitted, enable this if you're debugging, otherwise, just ignore
  
  // verify we get the right ACK
  digitalWrite(clockPin, HIGH);
  pinMode(dataPin, INPUT);
  ack = digitalRead(dataPin);
  #ifdef WIREDTEST
  if (ack != LOW)
    Serial.println("ACK error 0");
  #endif
  digitalWrite(clockPin, LOW);
  ack = digitalRead(dataPin);
  #ifdef WIREDTEST
  if (ack != HIGH)
    Serial.println("ACK error 1");
  #endif
  */
}

// wait for the SHTx answer
void waitForResultSHT(int dataPin) {
  int ack;

  pinMode(dataPin, INPUT);
  for(int i=0; i<100; ++i) {
    delay(10);
    ack = digitalRead(dataPin);
    if (ack == LOW)
      break;
  }
  /* 
  // For debugging SHT issues
  #ifdef WIREDTEST
  if (ack == HIGH)
    Serial.println("ACK error 2");
  #endif
  */
}

// get data from the SHTx sensor
int getData16SHT(int dataPin, int clockPin) {
  int val;

  // get the MSB (most significant bits)
  pinMode(dataPin, INPUT);
  pinMode(clockPin, OUTPUT);
  val = shiftIn(dataPin, clockPin, 8);
  val *= 256; // this is equivalent to val << 8;
  
  // send the required ACK
  pinMode(dataPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
  
  // get the LSB (less significant bits)
  pinMode(dataPin, INPUT);
  val |= shiftIn(dataPin, clockPin, 8);
  return val;
}

// skip CRC data from the SHTx sensor
void skipCrcSHT(int dataPin, int clockPin) {
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
}

#ifdef WINDRAIN
/* Wind and Weather sensor routines */

//=======================================================
// Interrupt handler for anemometer. Called each time the reed
// switch triggers (one revolution).
//=======================================================
void countAnemometer() {
   numRevsAnemometer++;
   windEvent = true;
}

//=======================================================
// Find vane direction.
//=======================================================
void calcWindDir() {
   int val, ref;
   byte x, reading;
   
   /* 
      This takes into account that the weather station may not be plugged into a constant voltage 5V power suppply.
      With this compensation, You can run this off a 3.3V board, and/or a battery-powered station with varying
      source voltage.
      Also, no direction compensation is done, you need to point the tip of the wind vane North. That means the pointed tip
      or arrow needs to point north.
      Update: The "read PIN_REF voltage" didn't work on a Fio or a Pro Mini. More work to do there, but leaving the original
              code in, in case I get some brilliant idea.
   */

   //ref = analogRead(PIN_REF); // Get reference voltage (voltage the board is running on right now)
   val = analogRead(PIN_VANE);
   //val = map (val, 0, ref, 0, 1023); // Map wind vane reading by reference voltage, since values were based on 5V
   val >>=2;                        // Shift to 255 range
   reading = val;

   // Look the reading up in directions table. Find the first value
   // that's >= to what we got.
   for (x=0; x<NUMDIRS; x++) {
      if (adc[x] >= reading)
         break;
   }
   windDirection = cdc[x];
   
   //Handy for debugging and making sure you're getting the right orentation
   //Serial.println(reading, DEC);
   //x = x % NUMDIRS;   // Adjust for orientation
   //Serial.print("  Dir: ");
   //Serial.println(strVals[x]);
}


//=======================================================
// Calculate the wind speed, and display it (or log it, whatever).
// 1 rev/sec = 1.492 mph
//=======================================================
void calcWindSpeed() {
   int x, iSpeed;
   // This will produce mph * 10
   // (didn't calc right when done as one statement)
   long speed = 14920;
   speed *= numRevsAnemometer;
   speed /= MSECS_CALC_WIND_SPEED;
   iSpeed = speed;         // Need this for formatting below
   windSpeed = speed / 10.0;
   if (windSpeed > windGust)
     windGust = windSpeed;

   //Serial.print(" Revs: ");
   //Serial.println(numRevsAnemometer);
   //Serial.print("Wind speed: ");
   x = iSpeed / 10;
   //Serial.print(x);
   //Serial.print('.');
   x = iSpeed % 10;
   //Serial.print(x);

   numRevsAnemometer = 0;        // Reset counter
}

//======================================================= 
// Interrupt handler for rainmeter. Called each time the reed 
// switch triggers (one tip of bucket). 
//======================================================= 
void countRainmeter() 
{ 
  static unsigned long last_millis = 0; 
  unsigned long m;
  
  if (windEvent) // If we're in a wind event, we're running in real time and not sleeping
     m = millis();
  else // nint is incremented every time watchdog goes off. See initialization in setup for details
     m = nint * nintMillis; 
  if (m - last_millis < 200){ 
    // ignore interrupt: probably a bounce problem 
  } else{ 
    numBuckettip++; 
    rainEvent = true;
    //Serial.print("Rain tips: ");
    //Serial.print(numBuckettip); 
    //Serial.print(" ");
  } 
  last_millis = m; 
}

/* End of Wind and Weather sensor routines */
#endif //WINDRAIN

//****************************************************************  
// set system into the sleep state 
// system wakes up when wtchdog is timed out
void system_sleep() {

  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System sleeps here

    sleep_disable();                     // System continues execution here when watchdog timed out 
    sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON

}

//****************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
//  Serial.println(ww);


  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);


}
//****************************************************************  
// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect) {
  f_wdt=1;  // set global flag
}

