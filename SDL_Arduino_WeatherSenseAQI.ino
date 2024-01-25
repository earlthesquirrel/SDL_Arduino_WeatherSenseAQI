// SDL_Arduino_WeatherSenseAQI
// SwitchDoc Labs January 2021

#define TXDEBUG
//#undef TXDEBUG

#include <JeeLib.h>
#include "MemoryFree.h"

// unique ID of this WeatherSenseAQI system - change if you have multiple WeatherSenseAQI systems
#define WEATHERSENSEAQIID 1

// WeatherSenseProtocol of 8 is SolarMAX LiPo   BatV < 7V
// WeatherSenseProtocol of 10 is SolarMAX LeadAcid   BatV > 7V LoRa version
// WeatherSenseProtocol of 11 is SolarMAX4 LeadAcid BatV > 7V
// WeatherSenseProtocol of 15 is WeatherSense AQI 433MHz
// WeatherSenseProtocol of 16 is WeatherSense ThunderBoard 433MHz
// WeatherSenseProtocol of 19 is WeatherSense Radiation
#define WEATHERSENSEPROTOCOL 19

// Software version
#define SOFTWAREVERSION 5

// unique ID of this WeatherSenseRadiation system - change if you have multiple WeatherSenseRadiation systems
#define WEATHERSENESTHBID 1
// Which WeatherSense TRadiation Protocol Version
#define WEATHERSENSEPROTOCOLVERSION 1

#define LED 13

// Powerdown Pin for HM3301 HQI
#define POWERDOWN 2

// Other Pins
#define WATCHDOG_1 5
#define TXPIN 8
#define RXPIN 9


// RANGETESTMODE puts the transmitter into a send every 15 seconds for range testing.  NOT FOR USE IN DEPLOYMENT!
//#define RANGETESTMODE
#undef RANGETESTMODE


//#ifdef RANGETESTMODE
// 15 second response
//#define SLEEPCYCLE 30000  // can't be less than 30000 without modification of sleep code
//#else
// 15 minute response
//#define SLEEPCYCLE (long)1000 * 60 * 15
//#endif

#define SLEEPCYCLE 30000


#include "Crc16.h"
//Crc 16 library (XModem)
Crc16 crc;


// AQI
#include <Seeed_HM330X.h>
HM330X hm330x;


uint8_t buf[30];

// hm33301 pValue
uint16_t pValue[6];
uint16_t pValuePrevious[6];

// Now calculate EPA AQI
uint16_t EPAAQI;


// pvalue[0]= "PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
// pvalue[1]= "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
// pvalue[2]= "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
// pvalue[3]= "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
// pvalue[4]= "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
// pvalue[5]= "PM10 concentration(Atmospheric environment,unit:ug/m3): ",


HM330XErrorCode print_result(const char* str, uint16_t value) 
{
  if (NULL == str) {
    return ERROR_PARAM;
  }
  Serial.print(str);
  Serial.println(value);
  return NO_ERROR;
}

// parse buf with 29 uint8_t-data
HM330XErrorCode parse_result(uint8_t* data)
 {
  uint16_t value = 0;
  if (NULL == data) {
    return ERROR_PARAM;
  }
  for (int i = 2; i < 8; i++) {
    value = (uint16_t)data[i * 2] << 8 | data[i * 2 + 1];
    pValue[i - 2] = value;
    Serial.print("value =");
    Serial.println(value);
  }

  return NO_ERROR;
}

HM330XErrorCode parse_result_value(uint8_t* data)
 {
  if (NULL == data) {
    return ERROR_PARAM;
  }
  uint8_t sum = 0;
  for (int i = 0; i < 28; i++) {
    sum += data[i];
  }
  if (sum != data[28]) {
    //Serial.println("wrong checkSum!!!!");
    return ERROR_PARAM;
  }
  return NO_ERROR;
}

ISR(WDT_vect) {
  Sleepy::watchdogEvent();
}

#include <RH_ASK.h>
#include <avr/sleep.h>
#include <avr/power.h>

// the three channels of the INA3221 named for INA3221 Solar Power Controller channels (www.switchdoc.com)
#define LIPO_BATTERY_CHANNEL 1
#define SOLAR_CELL_CHANNEL 2
#define OUTPUT_CHANNEL 3

RH_ASK driver(2000, RXPIN, TXPIN);

unsigned long MessageCount = 0;

#include "avr/pgmspace.h"
#include <Time.h>
#include <TimeLib.h>


#include <Wire.h>

typedef enum {

  NO_INTERRUPT,
  IGNORE_INTERRUPT,
  SLEEP_INTERRUPT,
  ALARM_INTERRUPT,
  REBOOT
} wakestate;


// Device Present State Variables
bool HM3301_Present;

byte byteBuffer[200];  // contains string to be sent to RX unit

// State Variables

long TimeStamp;


// State Status (sans HM3301)
byte AuxA;

// AuxA has state information
// coded in bottom four bits of the byte
// 0CAB

// A = 1, IN3221 (Solar) Present, 0 not present
// B = 1, HM3301 Present, 0 not present
// C = 1, Voltage below 2.9, HM3301 not turned on


int protocolBufferCount;
wakestate wakeState;  // who woke us up?
long nextSleepLength;


int convert4ByteLongVariables(int bufferCount, long myVariable) 
{
  int i;

  union {
    long a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++) {
    byteBuffer[bufferCount] = thing.bytes[i];
    bufferCount++;
  }
  return bufferCount;
}

int convert4ByteFloatVariables(int bufferCount, float myVariable) 
{
  int i;

  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++) {
    byteBuffer[bufferCount] = thing.bytes[i];
    bufferCount++;
  }

  return bufferCount;
}


int convert2ByteVariables(int bufferCount, int myVariable) 
{
  union {
    int a;
    unsigned char bytes[2];
  } thing;

  thing.a = myVariable;

  byteBuffer[bufferCount] = thing.bytes[0];
  bufferCount++;
  byteBuffer[bufferCount] = thing.bytes[1];
  bufferCount++;

  return bufferCount;
}

int convert1ByteVariables(int bufferCount, int myVariable)
{
  byteBuffer[bufferCount] = (byte)myVariable;
  bufferCount++;
  return bufferCount;
}

int checkSum(int bufferCount) 
{
  unsigned short checksumValue;
  // calculate checksum
  checksumValue = crc.XModemCrc(byteBuffer, 0, 59);
#if defined(TXDEBUG)
  Serial.print(F("crc = 0x"));
  Serial.println(checksumValue, HEX);
#endif

  byteBuffer[bufferCount] = checksumValue >> 8;
  bufferCount++;
  byteBuffer[bufferCount] = checksumValue & 0xFF;
  bufferCount++;

  return bufferCount;
}


int buildProtocolMessage() 
{

  int bufferCount;
  bufferCount = 0;

  bufferCount = convert4ByteLongVariables(bufferCount, MessageCount);

  byteBuffer[bufferCount++] = WEATHERSENSEAQIID;  // WeatherSenseAQI unique ID
  byteBuffer[bufferCount++] = WEATHERSENSEPROTOCOL;  // Type of WeatherSense System
  byteBuffer[bufferCount++] = WEATHERSENSEPROTOCOLVERSION;  // WeatherSense AQI protocol version

  // HM3301 Data

  bufferCount = convert2ByteVariables(bufferCount, pValue[0]);
  bufferCount = convert2ByteVariables(bufferCount, pValue[1]);
  bufferCount = convert2ByteVariables(bufferCount, pValue[2]);
  bufferCount = convert2ByteVariables(bufferCount, pValue[3]);
  bufferCount = convert2ByteVariables(bufferCount, pValue[4]);
  bufferCount = convert2ByteVariables(bufferCount, pValue[5]);
  bufferCount = convert2ByteVariables(bufferCount, EPAAQI);

  bufferCount = convert4ByteFloatVariables(bufferCount, 0.0); // Load Voltage
  bufferCount = convert4ByteFloatVariables(bufferCount, 0.0); // Battery Voltage
  bufferCount = convert4ByteFloatVariables(bufferCount, 0.0); // Battery Current
  bufferCount = convert4ByteFloatVariables(bufferCount, 0.0); // Load Current
  bufferCount = convert4ByteFloatVariables(bufferCount, 0.0); // Solar Panel Voltage
  bufferCount = convert4ByteFloatVariables(bufferCount, 0.0); // Solar Panel Curent

  byteBuffer[bufferCount] = AuxA | SOFTWAREVERSION << 4;  // Aux + Software Version
  bufferCount++;

  return bufferCount;
}


// AQI Calculation
#define AMOUNT_OF_LEVELS 6

int get_grid_index_(uint16_t value, int array[AMOUNT_OF_LEVELS][2]) 
{
  for (int i = 0; i < AMOUNT_OF_LEVELS; i++) {
    if (value >= array[i][0] && value <= array[i][1]) {
      return i;
    }
  }
  return -1;
}

int index_grid_[AMOUNT_OF_LEVELS][2] = { { 0, 51 }, { 51, 100 }, { 101, 150 }, { 151, 200 }, { 201, 300 }, { 301, 500 } };
int pm2_5_calculation_grid_[AMOUNT_OF_LEVELS][2] = { { 0, 12 }, { 13, 35 }, { 36, 55 }, { 56, 150 }, { 151, 250 }, { 251, 500 } };
int pm10_0_calculation_grid_[AMOUNT_OF_LEVELS][2] = { { 0, 54 }, { 55, 154 }, { 155, 254 }, { 255, 354 }, { 355, 424 }, { 425, 604 } };


int calculate_index_(uint16_t value, int array[AMOUNT_OF_LEVELS][2]) 
{
  int grid_index = get_grid_index_(value, array);
  int aqi_lo = index_grid_[grid_index][0];
  int aqi_hi = index_grid_[grid_index][1];
  int conc_lo = array[grid_index][0];
  int conc_hi = array[grid_index][1];

  return (((float)(aqi_hi - aqi_lo) / (float)(conc_hi - conc_lo)) * (value - conc_lo) + aqi_lo);
}


unsigned int get_aqi(unsigned int pm2_5_value, unsigned int pm10_0_value) 
{
  int pm2_5_index = calculate_index_(pm2_5_value, pm2_5_calculation_grid_);
  int pm10_0_index = calculate_index_(pm10_0_value, pm10_0_calculation_grid_);

  return (pm2_5_index < pm10_0_index) ? pm10_0_index : pm2_5_index;
}


void printStringBuffer() 
{
  int bufferLength;

  bufferLength = protocolBufferCount;
  int i;
  for (i = 0; i < bufferLength; i++) {
    Serial.print(F("i="));
    Serial.print(i);
    Serial.print(F(" | "));
    Serial.println(byteBuffer[i], HEX);
  }
}


void return2Digits(char returnString[], char* buffer2, int digits) 
{
  if (digits < 10)
    sprintf(returnString, "0%i", digits);
  else
    sprintf(returnString, "%i", digits);

  strcpy(returnString, buffer2);
}


void ResetWatchdog() 
{
  digitalWrite(WATCHDOG_1, LOW);
  delay(200);
  digitalWrite(WATCHDOG_1, HIGH);
}

void setup() 
{

  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  delay(200);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  delay(200);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  delay(200);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  delay(200);

  Serial.begin(115200);  // TXDEBUGging only
  Wire.begin();

  pinMode(WATCHDOG_1, OUTPUT);
  digitalWrite(WATCHDOG_1, HIGH);
  ResetWatchdog();

  AuxA = 0x00;
  // turn on USB Power for power check.

  Serial.println(F(">>>>>>>>>><<<<<<<<<"));
  Serial.println(F("WeatherSense AQI"));
  Serial.println(F(">>>>>>>>>><<<<<<<<<"));

  Serial.print(F("Software Version:"));
  Serial.println(SOFTWAREVERSION);

  Serial.print(F("Unit ID:"));
  Serial.println(WEATHERSENSEAQIID);

  if (!driver.init()) 
  {
    Serial.println(F("init failed"));
    while (1);
  }

  AuxA = AuxA & 0XFB;

  delay(1000);
  if (hm330x.init()) 
  {
    Serial.println(F("HM330X init failed!!!"));
    Serial.println(F("HM3301 Not Present"));
    HM3301_Present = false;
  } 
  else 
  {
    HM3301_Present = true;
    Serial.println(F("HM3301 Present"));
    // State Variable
    AuxA = AuxA | 0X01;
  }

  // initialize HM3301 data arrays
  int i;

  for (i = 0; i < 6; i++) 
  {
    pValue[i] = 0;
    pValuePrevious[i] = 0;
    ;
  }

  // setup initial values of variables
  wakeState = REBOOT;
  nextSleepLength = SLEEPCYCLE;

  TimeStamp = 0;
}

void readHM3301() 
{
  // hm33301
  if (hm330x.read_sensor_value(buf, 29)) 
  {
    Serial.println("HM330X read result failed!!!");
  }

  HM330XErrorCode hm3301error;
  hm3301error = parse_result_value(buf);
  parse_result(buf);

  if (hm3301error == NO_ERROR) 
  {
    // pValue set in parse_result_value
    int i;
    for (i = 0; i < 6; i++) 
    {
      pValuePrevious[i] = pValue[i];
    }
    Serial.println("HM3301 NO_ERROR");

  } 
  else 
  {
    // use previous read

    int i;
    for (i = 0; i < 6; i++) 
    {
      pValue[i] = pValuePrevious[i];
    }
    Serial.println("HM3301 ERROR");
  }
  //EPAAQI = get_aqi(pValue[1], pValue[2]);
  EPAAQI = get_aqi(pValue[4], pValue[5]);  // Atmospheric
}

void loop() 
{
  ResetWatchdog();
  // Only send if source is SLEEP_INTERRUPT
#if defined(TXDEBUG)
  Serial.print(F("wakeState="));
  Serial.println(wakeState);
#endif

if ((wakeState == SLEEP_INTERRUPT) || (wakeState == REBOOT))
 {
    wakeState = NO_INTERRUPT;
    TimeStamp = millis();

    ResetWatchdog();
    delay(10000);  // stablize fan
    readHM3301();
    ResetWatchdog();

    // check for bad read.  If bad (all zeros) read again

    int i;
    bool readGood = false;
    for (i = 0; i++; i < 6) 
    {
      if (pValue[i] != 0) 
      {
        readGood = true;
        break;
      }
#if defined(TXDEBUG)
      Serial.println(F("Bad HM3301 read - retrying"));
#endif
    }
    if (readGood == false)
    {
      readHM3301();
    }
    AuxA = AuxA & 0XFB;


#if defined(TXDEBUG)
    Serial.println(F("###############"));
    Serial.print(F("MessageCount="));
    Serial.println(MessageCount);
    Serial.print(F("STATUS - WeatherSenseProtocol:"));
    Serial.println(WEATHERSENSEPROTOCOL);
    // pvalue[0]= "PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
    // pvalue[1]= "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
    // pvalue[2]= "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
    // pvalue[3]= "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
    // pvalue[4]= "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
    // pvalue[5]= "PM10 concentration(Atmospheric environment,unit:ug/m3): ",

    Serial.print(F("PM1.0 Standard (ug/m3)="));
    Serial.println(pValue[0]);
    Serial.print(F("PM2.5 Standard (ug/m3)="));
    Serial.println(pValue[1]);
    Serial.print(F("PM10 Standard (ug/m3)="));
    Serial.println(pValue[2]);
    Serial.print(F("PM1.0 Atmospheric (ug/m3)="));
    Serial.println(pValue[3]);
    Serial.print(F("PM2.5 Atmospheric (ug/m3)="));
    Serial.println(pValue[4]);
    Serial.print(F("PM10 Atmospheric (ug/m3)="));
    Serial.println(pValue[5]);
    Serial.print(F("EPA AQI="));
    Serial.println(EPAAQI);

    Serial.print(F(" Currentmillis() = "));
    Serial.println(millis());
    Serial.print(F("  AuxA State:"));
    Serial.print(AuxA);
    Serial.print(F(" "));
    Serial.println(AuxA, HEX);

    Serial.println(F("###############"));
#endif

    // Now send the message in the current protocol and send.
    int bufferLength;

    Serial.println(F("----------Sending packets----------"));
    bufferLength = buildProtocolMessage();

    // Send a message
    driver.send(byteBuffer, bufferLength);
    Serial.println(F("----------After Sending packet----------"));

    for (int i = 0; i < bufferLength; i++) 
    {
      Serial.print(" ");
      if (byteBuffer[i] < 16) {
        Serial.print(F("0"));
      }
      Serial.print(byteBuffer[i], HEX);  //  write buffer to hardware serial port
    }
    Serial.println();

    if (!driver.waitPacketSent(6000)) 
    {
      Serial.println(F("Timeout on transmission"));
      // re-initialize board
      if (!driver.init()) {
        Serial.println(F("init failed"));
        while (1)
          ;
      }
      Serial.println(F("----------Board Reinitialized----------"));
    }

    Serial.println(F("----------After Wait Sending packet----------"));
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);

    Serial.print(F("freeMemory()="));
    Serial.println(freeMemory());
    Serial.print(F("bufferlength="));
    Serial.println(bufferLength);

    MessageCount++;
    Serial.println(F("----------Packet Sent.  Sleeping Now----------"));
  }


  ResetWatchdog();

  if (wakeState != REBOOT)
    wakeState = SLEEP_INTERRUPT;
  long timeBefore;
  long timeAfter;
  timeBefore = millis();
#if defined(TXDEBUG)
  Serial.print(F("timeBeforeSleep="));
  Serial.println(timeBefore);
  Serial.print(F("nextSleepLength="));
  Serial.println(nextSleepLength);
#endif
  delay(100);

  // set  with adjustments from watchdog (adds 200ms to each long cycle)  200ms
  // 30000 second inner loop


  for (long j = 0; j < (nextSleepLength / 30400); ++j) 
  {
    ResetWatchdog();
    for (long i = 0; i < 30000 / 16; ++i) 
    {
      Sleepy::loseSomeTime(16);
    }
  }

  wakeState = SLEEP_INTERRUPT;

#if defined(TXDEBUG)
  Serial.print(F("Awake now: "));
#endif
  timeAfter = millis();
#if defined(TXDEBUG)
  Serial.print(F("timeAfterSleep="));
  Serial.println(timeAfter);
  Serial.print(F("SleepTime = "));
  Serial.println(timeAfter - timeBefore);

  Serial.print(F("Millis Time: "));
#endif
  long time;
  time = millis();
#if defined(TXDEBUG)
  //prints time since program started
  Serial.println(time / 1000.0);
  Serial.print(F("2wakeState="));
  Serial.println(wakeState);
#endif
}
