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

#define numOfBLEPackets 32
#define numOfPacketsPerADCPin 4

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

#define ON_BOARD_LED 13



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
    Serial.print(Vrms);
    if (i < numOfBLEPackets/numOfPacketsPerADCPin - 1)
    {
      Serial.print(",");
    }
  }
  Serial.println("");
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
  Serial.println("C1, C2, C3, C4, C5, C6, C7, C8");
}

void loop()
{      
      calculate_battery_level();
      update_voltages();
}
