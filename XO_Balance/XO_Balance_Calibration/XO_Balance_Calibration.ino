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

#define numOfADCs 12

// ADC pin definitions
const int batteryLevelPin {A13};
const int adcPin[12] {A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12}; // The order of the adcs listed here determines the order of the BLE Vrms outputs.

// setting PWM properties
const int ledPin {26}; // 32 corresponds to GPIO16. (PWM)
const int freq {1000};
const int ledChannel {0};
const int resolution {1};

//placeholder Variables
int adc = 100; // placeholder voltage
float Vrms[12] = {0,0,0,0,
                  0,0,0,0,
                  0,0,0,0};
float batteryLevel = 0.0;
float batteryLevelInitial = 0.0;
float VrmsSampleLimit = 500;
float alpha = 0.2; // value between 0 (too filtered) and 1 (no filter). 
float Vnew = 0.0;

float deltaV = 0.0;
float fudgeFactor = 1.112;

//Feather and Pad Specific Specifications
//float c[numOfADCs];  




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
    Serial.print(Vrms[i]);
    if (i < numOfADCs - 1)
    {
      Serial.print(",");
    }
  }
  Serial.println("");
}

void calculate_battery_level()
{
  batteryLevel = (analogRead(batteryLevelPin)/4095.0)*2.0*3.3*fudgeFactor;
}

void fill_voltage_history()
{
  for (int j = 0; j < numOfADCs; j++)
  {
    Vrms[j] = Vrms_calculation(adcPin[j]) + deltaV;
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

  fill_voltage_history();
}

void loop()
{      
      calculate_battery_level();
      update_voltages();
}
