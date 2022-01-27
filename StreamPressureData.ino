/*
Max Tree
XO-NANO
1/26/22

This code reads pressure sensor data output, calculates  the root-mean-square
of the data, filters the rms voltages, and then the output is output from the
Arduino Uno by UART. Use the Serial Plotter or Serial Monitor tools to view
the live stream data.

An extra feature of this code is an LED setup. Set the lowPressureRMSThresh
and the highPressureRMSThresh to determine the pressure at which each LED
will light.

This code is designed for an Arduino Uno.

For the connection diagram between an Arduino Uno and an XO-NANO pressure
sensor, see the User Guide.dox in Dev-Kit-Resources.
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
/****** End User Inputs Section ******/


/* Initialize Variables */
float sensorValue = 0;
float Vrms = 0;
float Vrmsd_1 = 0; // "d_1 stands for delayed 1.
float Vrmsf = 0;   // "f" stands for filtered.


void setup() {
  Serial.begin(9600); // Set Baud rate.
  int PWMPin = 9;
  analogWrite(PWMPin, 127); //50% Duty Cycle.
  pinMode(lowPressureLED, OUTPUT);
  pinMode(highPressureLED, OUTPUT);
}


float calculate_Vrms()
{
  Vrms = 0;
  for (int i = 0; i < VrmsSampleLimit; i++)
  {
    sensorValue =  analogRead(sensorPin);
    Vrms = Vrms + sensorValue*sensorValue;    
  }

  Vrms = Vrms/VrmsSampleLimit;
  Vrms = sqrt(Vrms);

  return Vrms;
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

void loop()
{
  Vrms = calculate_Vrms();
  Vrmsf = lpf(cutoffFreq, sampPeriod, Vrms, Vrmsd_1, Vrmsf);
  Serial.println(Vrmsf);
  LED_check(Vrmsf);
  //update the delayed terms.
  Vrmsd_1 = Vrms;
}
