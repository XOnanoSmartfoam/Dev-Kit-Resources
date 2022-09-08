/***********************************************************************************
XO-NANO Smartfoam
Max Tree
May 26, 2022

This program uses an ESP32 Feather to send load values of an XO-NANO pressure
map to the XOnanoPressure app. The app is programmed to recognize the UUID and
device name given in this code. This code is based on BLE_uart.ino example for
ESP32 BLE Arduino. This code accounts for the BLE module connection to ADC2.
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

// Load calibration coefficients, board 1001
float a[numOfADCs] { 0.0000088582, 0.0000165177, 0.0000128345, -0.0000208971, 0.0000077448, 0.0000072655, 0.0000600967, 0.0000117620, 0.0001835388, 0.0002829693, 0.0000402972, -0.0000303918 };
float b[numOfADCs] { -0.0516251081, -0.1110431717, -0.0865990916, 0.1581694675, -0.0479030777, -0.0385962325, -0.4028802400, -0.0708279905, -1.4296996866, -2.2053256266, -0.2833612871, 0.2373038712 };
float c[numOfADCs] { 93.3637744924, 245.0688512084, 191.5233790256, -400.0099395714, 94.5061575365, 57.8838238556, 892.7541355077, 135.2549881506, 3707.3555963383, 5723.7995186534, 660.2279197896, -616.5649501638 };
float d[numOfADCs] { -49131.8886994032, -176736.4892071899, -138081.8637335922, 338015.5562218285, -57881.0278690943, -16988.8833092164, -652751.1728562497, -79199.9419754249, -3199928.7399219768, -4947068.7175951321, -509189.1844596136, 533136.7838812321 };
float VSS[numOfADCs] { 2459.58, 2512.4, 2547.02, 2537.66, 2488.25, 2487.08, 2458.19, 2459.78, 2690.94, 2669.76, 2547.0, 2502.34 };
float balanceCoefficient = 0.55; // For balancing with board #1005.

// ADC pin definitions
const int batteryLevelPin {A13};
const int adcPin[12] {A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12}; // The order of the adcs listed here determines the order of the BLE Vrms outputs.

float calibrationArea = 54.52;
int switchLoad = 50.0; // lbs
float Area1 = 4.871;
float Area2 = 2.605;

const int lenOfADC2 = 7;
const int adc2[lenOfADC2] = {A1, A5, A6, A8, A10, A11, A12};
float ADC2BLECorrectionFactor = 40;
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


float ADC2_check(float V, int index)
{
  for(int i; i < lenOfADC2; i++)
  {
      if (adcPin[index] == adc2[i])
      {
        V = V + ADC2BLECorrectionFactor;
        return V;
      }
  }
  return V;
}


void update_voltages()
{
  for(int i = 0; i < numOfADCs; i++)
  {
    Vnew = Vrms_calculation(adcPin[i]); // + deltaV;
    Vnew = ADC2_check(Vnew, i);
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
//    Serial.print(load); // Voltage level check. Note that ADC2's voltage readings shift with the use of the BLE module.
//    if (i < numOfADCs - 1)
//    {
//      Serial.print(",");
//    }
  }
//  Serial.println("");
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
    Vrms[j] = ADC2_check(Vrms[j], j);
  }
  for(int i = 0; i < numOfADCs; i++) // get the filter started
  {
    Vnew = Vrms_calculation(adcPin[i]); // + deltaV;
    Vnew = ADC2_check(Vnew, i);
    Vrms[i] = (Vnew*alpha) + ((1-alpha)*Vrms[i]); // active, low pass filter
  }
  for(int i = 0; i < numOfADCs; i++) // this is a repeat to allow the filter a chance to get warmed up.
  {
    Vnew = Vrms_calculation(adcPin[i]); // + deltaV;
    Vnew = ADC2_check(Vnew, i);
    Vrms[i] = (Vnew*alpha) + ((1-alpha)*Vrms[i]); // active, low pass filter
  }
}


void initialize_BLE()
{
// Create the BLE Device
BLEDevice::init("XOBalanceL");

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
