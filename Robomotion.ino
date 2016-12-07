
#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/*************************CHANGE THIS VALUE********************************/
#define ADAFRUITBLE_REQ 0
/**************************************************************************/

/*************************CHANGE THIS VALUE********************************/
#define ADAFRUITBLE_RDY 0  
/**************************************************************************/

/*************************CHANGE THIS VALUE********************************/
#define ADAFRUITBLE_RST 0
/**************************************************************************/

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

/*************************CHANGE THIS VALUE********************************/
char nameToBeAdvertised[]="DEVICE_NAME"; // BLE advertising name - use this value in Robomotion app => name field
/**************************************************************************/



Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);// only one servo driver

#define VEHICLE_ESC_NEUTRAL_POSITION  1500  
#define VEHICLE_LEFTorRIGHT_NEUTRAL_POSITION  1500

#define VEHICLE_ESC_MIN  1000            // this is the 'minimum' pulse length count (out of 4096)
#define VEHICLE_ESC_MAX  2000            // this is the 'maximum' pulse length count (out of 4096)

#define VEHICLE_LEFTorRIGHT_MIN  1000    // this is the 'minimum' pulse length count (out of 4096)
#define VEHICLE_LEFTorRIGHT_MAX  2000    // this is the 'maximum' pulse length count (out of 4096)


#define servonum_vehicle_sterring 1      // left/right servo number on shield
#define serwonum_ESC  2                  // forward/backward servo number on shield

uint8_t vehicleSterringValue = 1500;     //vehicle left/right - start with neutral value
uint8_t vehicleESCValue = 1500;          //vehicle ESC - start with neutral value

const uint8_t vehicleStopKey = 0x07;     // STOP key
const uint8_t vehicleSterringKey = 0x04; // vehicle left/right key
const uint8_t vehicleESCKey = 0x05;      // vehicle ESC key
const uint8_t authValueKey = 0x57;       // password key


/*************************CHANGE THIS VALUE********************************/
const uint8_t authValue = 0x2A;          //CHANGE PASSWORD - DEFAULT IS "42" (hexadecimal "2A") - use this value in Robomotion app => password field
/**************************************************************************/


bool isAuthorized = false; 
const char isAuthorisedResponse[4]= {0x12, 0x43, 0x56, 0x77};
const uint8_t isAuthorisedResponseCharLength = 4;
const char isNOTAuthorisedResponse[4]= {0xF2, 0x4F, 0xF6, 0xF7};
const uint8_t isNOTAuthorisedResponseCharLength = 4;

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;
/**************************************************************************/
/*
    This function is called whenever select ACI events happen
*/
/**************************************************************************/
void aciCallback(aci_evt_opcode_t event)
{
  switch(event)
  {
    case ACI_EVT_DEVICE_STARTED:
      Serial.println(F("Advertising started"));
      break;
    case ACI_EVT_CONNECTED:
      Serial.println(F("Connected!"));
      break;
    case ACI_EVT_DISCONNECTED:
      Serial.println(F("Disconnected or advertising timed out"));
      stopVehicle();
      isAuthorized = false;
      break;
    default:
      break;
  }
}

/**************************************************************************/
/*
    This function is called whenever data arrives on the RX channel
*/
/**************************************************************************/
void rxCallback(uint8_t *buffer, uint8_t len)
{

  for(int i=0; i<len; i++)
  {
    byte startByte = buffer[i];
    if(startByte==0x1F){
      i++; 
    byte singleByte = buffer[i];
    uint8_t valueByte;
    switch(singleByte){
    
      case vehicleStopKey:
          Serial.println("vehicleStop");
          Serial.println(isAuthorized);
          i++;
          valueByte = buffer[i];
          if(isAuthorized)
          {
              stopVehicle();
          }
          break;
          
    case vehicleSterringKey:
          Serial.println("vehicle left/right servo");
          i++;
          valueByte = buffer[i];
          Serial.println(valueByte);
          if(isAuthorized)
          {
            if(valueByte!=vehicleSterringValue){
              vehicleTurnLeftOrRight(valueByte);
            }
          }
          break;
    
    case vehicleESCKey:
          Serial.println("ESC-value");
          i++;
          valueByte = buffer[i];
          Serial.println(valueByte);
          if(isAuthorized)
          {
            if(valueByte!=vehicleESCValue){
              vehicleMoveForwardOrBackward(valueByte);
            }
          }
          break;
          
    case authValueKey:
          Serial.println("AUTHORIZATION...");
          i++;
          valueByte = buffer[i];
//          Serial.println(valueByte,HEX);
//          Serial.println(authValue,HEX);
          if(valueByte==authValue){
            uint8_t* pointer; 
            pointer =(uint8_t*)&isAuthorisedResponse;
            BTLEserial.write(pointer, isAuthorisedResponseCharLength);
            isAuthorized = true;
            Serial.println("***AUTHORIZED***");
          }else{
            uint8_t* pointer; 
            pointer =(uint8_t*)&isNOTAuthorisedResponse;
            BTLEserial.write(pointer, isNOTAuthorisedResponseCharLength);
            Serial.println("WRONG PASSWORD");
          }
          break;    
      }
    }
  }
}

/**************************************************************************/
/*
    Move management
*/
/**************************************************************************/

/*********************************VEHICLE**************************************/
void setServoPulse(uint8_t n, double pulse) 
{
  double pulselength;
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  pulselength /= 4096;  // 12 bits of resolution
  pulse /= pulselength;
  pwm.setPWM(n, 0, pulse);
}

void vehicleMoveForwardOrBackward(double goPulseValue)
{
  int valueToBeSet = goPulseValue*4+1000;
  if(valueToBeSet<VEHICLE_ESC_MAX && valueToBeSet>VEHICLE_ESC_MIN){
     setServoPulse(serwonum_ESC, valueToBeSet);
  }
}

void vehicleTurnLeftOrRight(double turnPulseValue)
{
  int valueToBeSet = turnPulseValue*4+1000;
  if(valueToBeSet<VEHICLE_LEFTorRIGHT_MAX && valueToBeSet>VEHICLE_LEFTorRIGHT_MIN){
    setServoPulse(servonum_vehicle_sterring, valueToBeSet);
  }
}

void stopVehicle()
{
  setServoPulse(serwonum_ESC, VEHICLE_ESC_NEUTRAL_POSITION);
  setServoPulse(servonum_vehicle_sterring, VEHICLE_LEFTorRIGHT_NEUTRAL_POSITION);
}


/**************************************************************************/
/*
    SETUP
*/
/**************************************************************************/
void setup(void)
{ 
  Serial.begin(9600);
  #ifdef ESP8266
/*************************CHANGE THIS VALUE********************************/
  Wire.pins(2, 14);   // ESP8266 can use any two pins, such as SDA to #2 and SCL to #14
/**************************************************************************/
  #endif

  pwm.begin();
  pwm.setPWMFreq(60);
  stopVehicle();
  BTLEserial.setDeviceName(nameToBeAdvertised);
  BTLEserial.setRXcallback(rxCallback);
  BTLEserial.setACIcallback(aciCallback);
  BTLEserial.begin();

}

/**************************************************************************/
/*
    MAIN LOOP
*/
/**************************************************************************/

void loop()
{
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();
}
