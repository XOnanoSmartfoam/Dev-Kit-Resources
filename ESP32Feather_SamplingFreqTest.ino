/***********************
 * Sampling Frequency Test
 * Max Tree
 * 6/13/22
 * 
 * This code utilizes ESP32 timers to find out the sampling frequency of the ESP32 Feather HUZZAH.
 * Note that the ADC must be configured before an accurate sampling frequency measurement can be
 * taken.
 * 
 * Answer:
 * Sampling frequency of the ESP32 Feather HUZZAH when sampling just a for loop pattern is
 * 17,400Hz
 * and calculating the RMS of the voltage (for 250 samples) is about 70 Hz
 * 
 * I used the following website's suggestion for accessing the timer: https://github.com/espressif/arduino-esp32/issues/4160
 */

void calc_sampling_freq_with_Vrms(float numOfSamples,const int pin, int adc)
{
  float Vrms = 0;

  double tInitial = esp_timer_get_time(); //fcn returns the amount of time in microseconds since the initial boot up of the ESP32

  //Run ADC
  for (int i = 0; i < int(numOfSamples); i++)
  {
    adc = analogRead(pin);
    Vrms += adc*adc; // perform Vrms calculation to symbolize pressure calculation requirements.
  }
  Vrms = Vrms/numOfSamples;
  Vrms = sqrt(Vrms);
  
  double tFinal = esp_timer_get_time();
  double t = tFinal-tInitial;
  Serial.print(t);
  Serial.println("us");

  float freq = 1*1000000/t; //1 because only one Vrms value is calculated in this code.
  Serial.print("With Vrms, Sampling Frequency is: ");
  Serial.println(freq);

  Serial.println("Finished. Restart code for additional readings");
}

void calc_sampling_freq_no_Vrms(float numOfSamples,const int pin, int adc)
{
  double tInitial = esp_timer_get_time(); //fcn returns the amount of time in microseconds since the initial boot up of the ESP32

  //Run ADC
  for (int i = 0; i < int(numOfSamples); i++)
  {
    adc = analogRead(pin);
  }
  
  double tFinal = esp_timer_get_time();
  double t = tFinal-tInitial;
  Serial.print(t);
  Serial.println("us");

  float freq = numOfSamples*1000000/t;
  Serial.print("Without Vrms, Sampling Frequency is: ");
  Serial.println(freq);

  Serial.println("Finished. Restart code for additional readings");
}

void setup() {
  Serial.begin(115200);
  delay(100); // Allow serial to begin
  //ADC Settings
  float numOfSamples = 250.0;
  const int pin = A0;
  int adc = 0;
  
  Serial.println("Start program");

  adc = analogRead(pin); // Get the adc configuration done.

  calc_sampling_freq_no_Vrms(numOfSamples, pin, adc);
  calc_sampling_freq_with_Vrms(numOfSamples, pin, adc);
}

void loop() {
  // put your main code here, to run repeatedly:

}
