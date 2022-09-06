/***********************************************************************************
XO-NANO Smartfoam
Max Tree
Sept 9, 2022

This program uses an ESP32 Feather to send load values of an XO-NANO pressure
map to the XOnanoPressure app. The app is programmed to recognize the UUID and
device name given in this code. This code is based on BLE_uart.ino example for
ESP32 BLE Arduino. This code does not account for the shift in voltage due to
the connection between the BLE module and ADC #2.
************************************************************************************/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define numOfBLEPackets 51
#define numOfPacketsPerADCPin 4
#define numOfADCs 12

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
/*************************************
User Specific Edit Section
*************************************/

// Load calibration coefficients, board 1003
float a[numOfADCs] { 0.0000455373, 0.0000105647, 0.0000314870, 0.0000075204, 0.0000289642, 0.0000125604, 0.0000715168, 0.0000636612, 0.0003680054, 0.0003890859, 0.0000785549, 0.0000757175 };
float b[numOfADCs] { -0.3122820107, -0.0671178449, -0.2195561615, -0.0460733002, -0.1985692097, -0.0688681043, -0.4828705346, -0.4438132392, -2.8690152755, -3.0524649597, -0.5654700448, -0.5335883771 };
float c[numOfADCs] { 709.5023649207, 137.1755609595, 507.0123350551, 88.6921581882, 449.9668846180, 110.1746790126, 1078.3324340133, 1027.0850951191, 7450.1803369540, 7977.0413438910, 1352.9076326433, 1248.7460131969 };
float d[numOfADCs] { -533480.4749274735, -88528.3098960236, -387282.0045713758, -51268.5401233318, -336423.4591822351, -41250.1949265743, -795235.9986742261, -788560.7845999129, -6443729.4076435249, -6943891.0649302304, -1075475.2774776416, -970033.2960594716 };
float VSS[numOfADCs] { 2482.57, 2502.62, 2534.59, 2528.01, 2500.23, 2497.16, 2484.45, 2475.12, 2674.5, 2673.37, 2540.4, 2521.01 };
float balanceCoefficient = 0.8; // For balancing with board #1008. The 1.55 is how much it missed my weight by originally.

// ADC pin definitions
const int batteryLevelPin {A13};
const int adcPin[12] {A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12}; // The order of the adcs listed here determines the order of the BLE Vrms outputs.

float calibrationArea = 54.52;
int switchLoad = 50.0; // lbs
float Area1 = 4.871;
float Area2 = 2.605;

/*************************************
End of User Specific Edit Section
*************************************/


//BLE variables
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

uint8_t txValue[numOfBLEPackets] {0,0,0,0,
                                  0,0,0,0,
                                  0,0,0,0,
                                  0,0,0,0,
                                  
                                  0,0,0,0,
                                  0,0,0,0,
                                  0,0,0,0,
                                  0,0,0,0,
                                  
                                  0,0,0,0,
                                  0,0,0,0,
                                  0,0,0,0,
                                  0,0,0,0,
                                  
                                  0,0,0};  // last three are for the battery



// setting LED PWM properties
const int ledPin {26};
const int freq {1000};
const int ledChannel {0};
const int resolution {1};

//placeholder Variables
int adc {100};
float Vrms[12] = {0,0,0,0,
                  0,0,0,0,
                  0,0,0,0};
float batteryLevel {0.0};
float batteryLevelInitial {0.0};
float VrmsSampleLimit {250};
float alpha = 0.2; // value between 0 (too filtered) and 1 (no filter).
float Vnew = 0.0;
float deltaV = {0.0};
float pressure = 0.0;
float load = 0.0;

//General Calibration Values
float batteryCalibrationFactor {1.112};


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



void encode_load(int index)
{
  txValue[index] = load/100;
  txValue[index + 1] = (load-(txValue[index]*100))/10;
  txValue[index + 2] = (load-(txValue[index]*100)-txValue[index + 1]*10)/1;
  txValue[index + 3] = int((load-(txValue[index]*100)-txValue[index + 1]*10-txValue[index + 2]*1)/0.1);
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


void update_voltages()
{
  for(int i = 0; i < numOfADCs; i++)
  {
    Vnew = Vrms_calculation(adcPin[i]); // + deltaV;
    Vrms[i] = (Vnew*alpha) + ((1-alpha)*Vrms[i]); // active, low pass filter
    //convert to pressure
    pressure = balanceCoefficient*(a[i]*Vrms[i]*Vrms[i]*Vrms[i] + b[i]*Vrms[i]*Vrms[i] + c[i]*Vrms[i] + d[i])/(calibrationArea);
    
    if ((pressure < switchLoad/calibrationArea) || (Vrms[i] > VSS[i])) // ignore stuff less than 1psi because the curve_fit is not quite right for low pressures.
    {
      pressure = 0.0;
    }
    if (i != 8 || i != 9)
    {
      load = pressure*Area1;
    }
    else
    {
      load = pressure*Area2;
    }
    encode_load(i*numOfPacketsPerADCPin);

  }
  //encode battery level 
  txValue[48] = batteryLevel/1;
  txValue[49] = int((batteryLevel-(txValue[48]*1))/0.1);
  txValue[50] = int((batteryLevel-(txValue[48]*1) - (txValue[49]*.1))/0.01);
}


void calculate_battery_level()
{
batteryLevel = (analogRead(batteryLevelPin)/4095.0)*2.0*3.3*batteryCalibrationFactor;
}


void fill_voltage_history()
{
  for (int j = 0; j < numOfADCs; j++) //initialize history
  {
    Vrms[j] = Vrms_calculation(adcPin[j]); // + deltaV;
  }
  for(int i = 0; i < numOfADCs; i++) // get the filter started
  {
    Vnew = Vrms_calculation(adcPin[i]); // + deltaV;
    Vrms[i] = (Vnew*alpha) + ((1-alpha)*Vrms[i]); // active, low pass filter
  }
  for(int i = 0; i < numOfADCs; i++) // this is a repeat to allow the filter a chance to get warmed up.
  {
    Vnew = Vrms_calculation(adcPin[i]); // + deltaV;
    Vrms[i] = (Vnew*alpha) + ((1-alpha)*Vrms[i]); // active, low pass filter
  }
}


void initialize_BLE()
{
// Create the BLE Device
BLEDevice::init("XOBalanceR");

// Create the BLE Server
pServer = BLEDevice::createServer();
pServer->setCallbacks(new MyServerCallbacks());

// Create the BLE Service
BLEService *pService = pServer->createService(SERVICE_UUID);

// Create a BLE Characteristic
pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_WRITE |
                    BLECharacteristic::PROPERTY_NOTIFY
                    );

pTxCharacteristic->addDescriptor(new BLE2902());

pService->start();
pServer->getAdvertising()->start();
Serial.println("Waiting a client connection to notify...");
}



void setup()
{
  Serial.begin(115200);
  initialize_BLE();
  
  // configure LED PWM
  ledcSetup(ledChannel, freq, 8);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, 127);
  
  calculate_battery_level();
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
