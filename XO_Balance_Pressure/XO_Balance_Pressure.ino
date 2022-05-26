/***********************************************************************************
  XO-NANO Smartfoam
  Max Tree
  May 26, 2022
  
  This program uses an ESP32 Feather to send pressure values of an XO-NANO pressure
  map to the XOnanoPressure app. The app is programmed to recognize the UUID and
  device name given in this code. This code is based on BLE_uart.ino example for
  ESP32 BLE Arduino.
************************************************************************************/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define numOfBLEPackets 48
#define numOfPacketsPerADCPin 4
#define numOfADCs 12

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
/*************************************
User Specific Edit Section
*************************************/


float a[numOfADCs] {0.0013274875, 0.0012393815, 0.0010340313333333333, 0.0010401863333333332, 1.0539999999999999e-05, 0.0004650485,
                    0.0008428140000000001, -9.254999999999999e-06, 0.0017017333333333334, 0.003818865333333333, 0.0012614384999999999, 0.001430699499999};
float b[numOfADCs] {-6.474206938, -6.066640789, -5.021333532333333, -5.086328079, -0.086797565, -2.246442914,
                    -3.9693308735, -0.085137536, -9.454282621666666, -19.986659506666665, -6.2566444919999995, -6.988277646};

// ADC pin definitions
const int batteryLevelPin {A13};
const int adcPin[12] {A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12}; // The order of the adcs listed here determines the order of the BLE Vrms outputs.
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
                                  0,0,0,0};

// setting PWM properties
const int ledPin {26}; // 32 corresponds to GPIO16. (PWM)
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
float VrmsSampleLimit {500};
float alpha = 0.2; // value between 0 (too filtered) and 1 (no filter).
float Vnew = 0.0;
float deltaV ={0.0};
float pressure = 0.0;

//General Calibration Values
float batteryCalibrationFactor {1.112};
float c[12] = {1,1,1,1,
                1,1,1,1,
                1,1,1,1}; // These are calculated below


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

void encode_pressure(int index)
{ 
  if (pressure < 0)
  {
    pressure = 0.0; 
  }
  
  txValue[index] = pressure/100;
  txValue[index + 1] = (pressure-(txValue[index]*100))/10;
  txValue[index + 2] = (pressure-(txValue[index]*100)-txValue[index + 1]*10)/1;
  txValue[index + 3] = (pressure-(txValue[index]*100)-txValue[index + 1]*10-txValue[index + 2]*1)/0.1;
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

void update_voltages()
{
  account_for_battery_level();
  for(int i = 0; i < numOfADCs; i++)
  {
    Vnew = Vrms_calculation(adcPin[i]) + deltaV;
    Vrms[i] = (Vnew*alpha) + ((1-alpha)*Vrms[i]); // active, low pass filter
    //convert to pressure
    pressure = (a[i]*Vrms[i]*Vrms[i] + b[i]*Vrms[i] + c[i])/(4);// The four is because the calibration was divided up by four sensors at a time.
    Serial.print(Vrms[i]);
    Serial.print(" ");
    encode_pressure(i*numOfPacketsPerADCPin);
  }
  Serial.println("");
}

void calculate_battery_level()
{
  batteryLevel = (analogRead(batteryLevelPin)/4095.0)*2.0*3.3*batteryCalibrationFactor;
}

void fill_voltage_history()
{
  for (int j = 0; j < numOfADCs; j++) //initialize history
  {
    Vrms[j] = Vrms_calculation(adcPin[j]) + deltaV;
  }
    for(int i = 0; i < numOfADCs; i++)
  {
    Vnew = Vrms_calculation(adcPin[i]) + deltaV;
    Vrms[i] = (Vnew*alpha) + ((1-alpha)*Vrms[i]); // active, low pass filter
  }
      for(int i = 0; i < numOfADCs; i++) // this is a repeat to allow the filter a chance to get warmed up.
  {
    Vnew = Vrms_calculation(adcPin[i]) + deltaV;
    Vrms[i] = (Vnew*alpha) + ((1-alpha)*Vrms[i]); // active, low pass filter
  }
}

void calc_c()
{
  for (int i = 0; i < numOfADCs; i++)
  {  
    c[i] = -a[i]*Vrms[i]*Vrms[i] - b[i]*Vrms[i];
  }
  // ADD specific pressure bed lines of code here for fine tuning.
//   c[6] = c[6] - 5; //specific to board 002
//   c[4] = c[4] - 1; //specific to board 002
}

void initialize_BLE()
{  
  // Create the BLE Device
  BLEDevice::init("Pressure Sense 12 Pads R");

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
}

void setup() {
  Serial.begin(115200);
  initialize_BLE();

  // configure LED PWM
  ledcSetup(ledChannel, freq, 8);
  ledcAttachPin(ledPin, ledChannel);
  ledcWrite(ledChannel, 127);

  calculate_battery_level();
  batteryLevelInitial = batteryLevel;

  fill_voltage_history();
  calc_c(); // get all calibration equations to the same starting point.
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
  Serial.println("Here");

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
