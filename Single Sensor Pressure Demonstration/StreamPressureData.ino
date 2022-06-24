/*
Max Tree
XO-NANO
1/26/22

Before running this code, calibrate your sensor with CalibrateXOnanoPressureSensor.ino.
The calibration script will give you the numbers you need for VrmsKnownLoad, knownLoad,
and VrmsZero.

This code reads pressure sensor data output, calculates  the root-mean-square
of the data, filters the rms voltages, and then the voltage is converted to
pressure and sent from the Arduino Uno by UART. Use the Serial Plotter or
Serial Monitor tools to view the live stream data.

An extra feature of this code is an LED setup. Set the lowPressureRMSThresh
and the highPressureRMSThresh to determine the pressure at which each LED
will light.

This code is designed for an Arduino Uno.

For the connection diagram between an Arduino Uno and an XO-NANO pressure
sensor, see Pressure Sensor Instructions.docx at
https://github.com/XOnanoSmartfoam/Dev-Kit-Resources/blob/main/Single%20Sensor%20Pressure%20Demonstration/CalibrateXOnanoPressureSensor.ino
*/

/****** User Inputs Section ******/
/* Arduino Settings */
int sensorPin = A0;
int lowPressureLED = 3;
int highPressureLED = 4;
//for the PWM pin, see setup() function.
/* XO-NANO Pressure Sensor Settings */
float VrmsSampleLimit = 500;
int lowPressureRMSThresh = 600;
int highPressureRMSThresh = 575;
float cutoffFreq = 1.0;  //Hz.
float sampPeriod = 0.093;  //s. This number is from the timestamp of the Arduino IDE Serial Monitor tool.
//From Calibration
float sensorSurfaceArea = ; //square inches
float knownLoad = ; //lb
float VrmsZero = ;
float VrmsKnownLoad = ;
//Quick Recalibrate Variable
float quickRecalibratePressure = 0; //psi. If what should be zero pressure reads a different number on the Serial Monitor, input that number here.
/****** End User Inputs Section ******/


/* Initialize Variables */
float sensorValue = 0;
float Vrms = 0;
float Vrmsd_1 = 0; // "d_1 stands for delayed 1.
float Vrmsf = 500;   // "f" stands for filtered. Start at high value to reduce filter warm up time.
float pressure = 0; //psi.
float m = (knownLoad/sensorSurfaceArea)/(VrmsKnownLoad-VrmsZero); // dPressure/dVrms


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

void LED_check(float Vrms)
{
  if (Vrms < highPressureRMSThresh)
  {
    digitalWrite(highPressureLED, HIGH);
    digitalWrite(lowPressureLED, LOW);
  }
  else if (Vrms < lowPressureRMSThresh)
  {
    digitalWrite(highPressureLED, LOW);
    digitalWrite(lowPressureLED, HIGH);
  }
  else
  {
    digitalWrite(highPressureLED, LOW);
    digitalWrite(lowPressureLED, LOW);
  }
}

float lpf(float fc, float Ts, float y, float yd_1, float yfd_1) // fc is cutoff freq (Hz), Ts is sampling period (s).
{                                                               // y is raw data and yf is filtered data and the d terms are delayed terms.
  return (fc*y + yfd_1/Ts)/(1/Ts+fc);                           //derived by converting lpf lapace domain into time domain and assuming yf_dot = (yf-yfd_1)/Ts.
}

float convert_to_pressure(float V)
{
  return ((V-VrmsZero)*m - quickRecalibratePressure);
}


void setup() {
  Serial.begin(9600); // Set Baud rate.
  int PWMPin = 9;
  analogWrite(PWMPin, 127); //50% Duty Cycle.
  pinMode(lowPressureLED, OUTPUT);
  pinMode(highPressureLED, OUTPUT);
  Vrmsd_1 = calculate_Vrms();
}

void loop()
{
  Vrms = calculate_Vrms();
  Vrmsf = lpf(cutoffFreq, sampPeriod, Vrms, Vrmsd_1, Vrmsf);
  pressure = convert_to_pressure(Vrmsf);
  Serial.println(pressure);
  LED_check(Vrmsf);
  //update the delayed terms.
  Vrmsd_1 = Vrms;
}
