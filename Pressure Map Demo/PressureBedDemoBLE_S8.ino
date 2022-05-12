/***********************************************************************************
  XO-NANO Smartfoam
  Max Tree
  May 11, 2022
  
  This program uses an ESP32 Feather to send pressure values of an XO-NANO pressure
  map to the XOnanoPressure app. The app is programmed to recognize the UUID and
  device name given in this code. This code is based on BLE_uart.ino example for
  ESP32 BLE Arduino.
************************************************************************************/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define numOfBLEPackets 32
#define numOfPacketsPerADCPin 4
#define numOfADCs 8

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define ON_BOARD_LED 13
/*************************************
User Specific Edit Section
*************************************/

float p[numOfADCs] {0.83,1.45,1.34,1.4,
                    1.35,1.23,1.3,0.93}; // pressure calibration scalers. Replace these numbers with your specific calibration scalers.

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
                                  0,0,0,0};

// ADC pin definitions
const int batteryLevelPin {A13};
const int adcPin[numOfADCs] {A2, A3, A4, A5, A6, A7, A8, A9}; // The order of the adcs listed here determines the order of the BLE Vrms outputs.

// setting PWM properties
const int ledPin {26}; // 32 corresponds to GPIO16. (PWM)
const int freq {1000};
const int ledChannel {0};
const int resolution {1};

//placeholder Variables
int adc {100}; // placeholder voltage
float Vrms {0.0};
float batteryLevel {0.0};
float batteryLevelInitial {0.0};
float VrmsSampleLimit {500};
const int avgSize {5};
float vHistory[numOfADCs][avgSize];
int vHIndex {0};
float deltaV ={0.0};
float pressure = 0.0;

//General Calibration Values
float batteryCalibrationFactor {1.112};
float a = 4.27*0.0001;
float b = -2.03;
float c[numOfADCs] {1,1,1,1,
            1,1,1,1}; // bringing all sensors to the same origin


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
    vHistory[i][vHIndex] = Vrms_calculation(adcPin[i]); //*c[i]
    Vrms = calc_history_avg(i); // for moving average
    Vrms = Vrms + deltaV;//battery level calc
    //convert to pressure
    //This calibratrion was performed with a 2"x2" platen
    pressure = p[i]*(a*Vrms*Vrms + b*Vrms + c[i])/(2*2);
    
    encode_pressure(i*numOfPacketsPerADCPin);
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
  batteryLevel = (analogRead(batteryLevelPin)/4095.0)*2.0*3.3*batteryCalibrationFactor;
}

void fill_voltage_history()
{
  for (int i = 0; i < avgSize; i++)
  {
    for (int j = 0; j < numOfADCs; j++)
    {
      vHistory[j][i] = Vrms_calculation(adcPin[j]);
    }
  }
}


void calc_c()
{
  for (int i = 0; i < numOfADCs; i++)
  {  
    c[i] = -a*calc_history_avg(i)*calc_history_avg(i) - b*calc_history_avg(i);
  }
  // ADD specific pressure bed lines of code here for fine tuning.
   c[6] = c[6] - 1.3; //specific to board 002
   c[4] = c[4] - 0.3; //specific to board 002
}

void initialize_BLE()
{  
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

  //Turn On System Power Indicator LED
  pinMode(ON_BOARD_LED, OUTPUT);
  digitalWrite(13, HIGH); // power on

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
