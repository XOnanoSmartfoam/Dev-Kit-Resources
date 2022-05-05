/***********************************************************************************
  Max Tree
  XO-NANO Smartfoam
  March 22, 2022
  
  This program uses an ESP32 Feather to read in voltages for eight different XO_NANO
  pressure sensors, calculate each root-mean-square voltage (Vrms), and then notify
  a service with a 32byte package (conatining every Vrms) via BLE constantly. The
  code is based on BLE_uart.ino example for ESP32 BLE Arduino. The ESP32 Feather
  with this code is named on line of this code that is:
  "BLEDevice::init("Pressure Sense");:
************************************************************************************/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//BLE variables
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define numOfBLEPackets 32
#define numOfPacketsPerADCPin 4
uint8_t txValue[numOfBLEPackets]  =  {0,0,0,0,
                                      0,0,0,0,
                                      0,0,0,0,
                                      0,0,0,0,
                                      0,0,0,0,
                                      0,0,0,0,
                                      0,0,0,0,
                                      0,0,0,0};

// ADC pin definitions
const int batteryLevelPin {A13};
const int adcPin[8] {A2, A3, A4, A5, A6, A7, A8, A9}; // The order of the adcs listed here determines the order of the BLE Vrms outputs.

// setting PWM properties
const int ledPin = 26; // 32 corresponds to GPIO16. (PWM)
const int freq = 1000;
const int ledChannel = 0;
const int resolution = 1;

//placeholder Variables
int adc = 100; // placeholder voltage
float Vrms = 0;
float batteryLevel = 0.0;
float batteryLevelInitial = 0.0;
float VrmsSampleLimit = 500;
const int avgSize = 5;
float vHistory[8][avgSize];
int vHIndex = 0;

float deltaV = 0.0;
float fudgeFactor = 1.112;

//Feather and Pad Specific Specifications
float c[8] {1,1,1,1,
            1,1,1,1};  


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
//#define CHARACTERISTIC_UUID_RX "832b68cc-c342-11ec-9d64-0242ac120002"
//#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define ON_BOARD_LED 13


class MyServerCallbacks: public BLEServerCallbacks
{
    void onConnect(BLEServer* pServer)
    {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer)
    {
      deviceConnected = false;
    }
};

void encode_Vrms(int index)
{  
  txValue[index] = Vrms/1000;
  txValue[index + 1] = (Vrms-(txValue[index]*1000))/100;
  txValue[index + 2] = (Vrms-(txValue[index]*1000)-txValue[index + 1]*100)/10;
  txValue[index + 3] = (Vrms-(txValue[index]*1000)-txValue[index + 1]*100-txValue[index + 2]*10)/1;
}

float Vrms_calculation(int pin)
{
  float V = 0;
  for (int i = 0; i < VrmsSampleLimit; i++)
  {
    adc = analogRead(pin);
    V += adc*adc;
  }
  
  V = V/VrmsSampleLimit;
  V = sqrt(V);
  return (V);
}

void account_for_battery_level() //the change in voltage of the battery dictates the change in voltage that 0psi will experience regardless of the impedance of the circuit.
{
  if ( batteryLevel <= 4.0)
  {
    deltaV = 650.36*(batteryLevelInitial-batteryLevel);
  }
  else
  {
    deltaV = 50.0*(batteryLevelInitial-batteryLevel);
  }
}

float calc_history_avg(int i)
{
  float avg = 0;
    for (int j = 0; j < avgSize; j++)
  {
      avg = avg + vHistory[i][j];
  }
  avg = avg/avgSize;
  return avg;
}

void update_voltages()
{
  account_for_battery_level();
  for(int i = 0; i < numOfBLEPackets/numOfPacketsPerADCPin; i++)
  {
    vHistory[i][vHIndex] = Vrms_calculation(adcPin[i])*c[i];
    Vrms = calc_history_avg(i); // for moving average
    Vrms = Vrms + deltaV;//battery level calc
//    Vrms = Vrms*c[i];//individual calibration fudge factors
    encode_Vrms(i*numOfPacketsPerADCPin);
  }
  if (vHIndex < avgSize-1)
  {
    vHIndex++;
  }
  else
  {
    vHIndex = 0;
  }
}

void calculate_battery_level()
{
  batteryLevel = (analogRead(batteryLevelPin)/4095.0)*2.0*3.3*fudgeFactor;
}

void fill_voltage_history()
{
  for (int i = 0; i < avgSize; i++)
  {
    for (int j = 0; j < 8; j++)
    {
      vHistory[j][i] = Vrms_calculation(adcPin[j])*c[j];
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // Create the BLE Device
  BLEDevice::init("Pressure Sense 8 Pads");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_WRITE  |
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());
                    
  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, 8);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);

  ledcWrite(ledChannel, 127);

  calculate_battery_level();
  batteryLevelInitial = batteryLevel;

  pinMode(ON_BOARD_LED, OUTPUT);
  digitalWrite(13, HIGH); // power on

  fill_voltage_history();
}

void loop()
{      
    if (deviceConnected)
    {
      calculate_battery_level();
      update_voltages();
      //BLE communication
      pTxCharacteristic->setValue(&txValue[0], numOfBLEPackets); // The first number must be unit8_t and the second is the size of the information being sent.
      pTxCharacteristic->notify();
    }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
        delay(500); // give the bluetooth stack the chance to get things ready
        Serial.println("Connection Lost.");
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected)
    {
    // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}
