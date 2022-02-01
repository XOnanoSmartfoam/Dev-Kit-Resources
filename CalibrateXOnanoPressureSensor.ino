/*
Max Tree
XO-NANO
2/1/22

This code computes the numbers needed for calibrating an XO-NANO pressure sensor.
Open the Serial Monitor and follow the instructions after uploading this code
onto your microcontroller.

This code is designed using an Arduino Uno.

For the connection diagram between an Arduino Uno and an XO-NANO pressure
sensor, see https://github.com/XOnanoSmartfoam/Dev-Kit-Resources.


INSTRUCTIONS
1. Find a flat object to distribute weight of known weights
2. Find a known weight (dumbell, weighed object)
3. Enter the known weight's value into the knownLoad variable
4. Enter the the value for the sensorSurfaceArea (2x2, 1x1, 0.5x.05)
5. Upload this code onto your microcontroller
6. Follow instructions that come to the Serial Monitor tool.

*/

/****** User Inputs Section ******/
/* Arduino Settings */
int sensorPin = A0;
//for the PWM pin, see setup() function.
/* XO-NANO Pressure Sensor Settings */
float VrmsSampleLimit = 500;
float cutoffFreq = 1.0;  //Hz.
float sampPeriod = 0.093;  //s. This number is from the timestamp of the Arduino IDE Serial Monitor tool.
float sensorSurfaceArea = ; //square inches
float knownLoad = ; //lb
/****** End User Inputs Section ******/


/* Initialize Variables */
float sensorValue = 0.0;
float Vrms = 500.0;
float Vrmsd_1 = 500.0; // "d_1 stands for delayed 1.
float Vrmsf = 500.0;   // "f" stands for filtered.
float VrmsSS = 0.0; // Steady state ADC output



float calculate_Vrms()
{
  float V = 0;
  for (int i = 0; i < VrmsSampleLimit; i++)
  {
    sensorValue =  analogRead(sensorPin);
    V = V + sensorValue*sensorValue;    
  }

  V = V/VrmsSampleLimit;
  V = sqrt(V);

  return V;
}

float lpf(float fc, float Ts, float y, float yd_1, float yfd_1) // fc is cutoff freq (Hz), Ts is sampling period (s).
{                                                               // y is raw data and yf is filtered data and the d terms are delayed terms.
  return (fc*y + yfd_1/Ts)/(1/Ts+fc);                           //derived by converting lpf lapace domain into time domain and assuming yf_dot = (yf-yfd_1)/Ts.
}

void check_for_known_pressure(float F)
{
  Serial.print("Enter any character when ");
  Serial.print(F);
  Serial.println("lb is evenly distributed on the XO-NANO pressure sensor.");
  while(Serial.available()==0);
  delay(100);
  while(Serial.available() > 0)Serial.read();
}

void record_ADC_output_known_pressure(float F)
{
  int filterWarmUp = 400;
  for( int i = 0; i < filterWarmUp; i++)
  {
    Vrms = calculate_Vrms();
    Vrmsf = lpf(cutoffFreq, sampPeriod, Vrms, Vrmsd_1, Vrmsf);
    //update the delayed terms.
    Vrmsd_1 = Vrms;
  }
  VrmsSS = Vrms;
  if (F == 0.0)
  {
    Serial.print("VrmsZero = ");
  }
  else
  {
    Serial.print("VrmsKnownLoad = ");
  }
  Serial.println(VrmsSS);
}

void calibrate_sensor()
{
  check_for_known_pressure(0.0);
  Serial.println("Wait while the sensor is calibrated.");
  record_ADC_output_known_pressure(0.0);
  
  check_for_known_pressure(knownLoad);
  Serial.println("Wait while the sensor is calibrated.");
  record_ADC_output_known_pressure(knownLoad);
  
  Serial.println("Done. Enter the values for knownLoad, sensorSurfaceArea, VrmsZero, and VrmsKnownLoad into StreamPressureData.ino");
}

void setup() {
  Serial.begin(9600); // Set Baud rate.
  int PWMPin = 9;
  analogWrite(PWMPin, 127); //50% Duty Cycle.
  Vrmsd_1 = calculate_Vrms();
  Serial.println("Make sure you have read the instructions at the top of the source code.");
  calibrate_sensor();
}

void loop()
{

}
