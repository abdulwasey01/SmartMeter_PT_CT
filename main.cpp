#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include "EmonLib.h"             // Include Emon Library
#define VOLT_CAL 48               
EnergyMonitor emon1;             // Create an instance


Adafruit_ADS1115 ads;
const float FACTOR=58; // factor for 100A/1V output CT
const float FACTOR1 = 29; //factor for50A/1V output CT

const float multiplier = 0.00005;

float getcurrent();
float getcurrent1();

void setup() {
  Serial.begin(115200);
                                //setting gain to get the output exanded to all bits
  ads.setGain(GAIN_FOUR);      // +/- 1.024V 1bit = 0.5mV
  ads.begin();
  
  // No need for VOLT_CAL when using ESP32 ADC, perform calibration separately if needed
  emon1.voltage(36, VOLT_CAL, 60);  // Voltage: input pin (ADC channel), calibration value (adjust as needed)

}
// Function to print the RMS value of the current.
void printMeasure(String prefix, float value, String postfix)
{
  Serial.print(prefix);
  Serial.print(value, 2); // to get 2 decimal places in the rms value
  Serial.println(postfix);
}
// loop that repeatedly gets the values from the both current transformers
void loop() {
  float currentRMS = getcurrent();
  float currentRMS1 = getcurrent1();
  emon1.calcVI(25, 1000);         // Calculate all. No.of half wavelengths (crossings), time-out
  float supplyVoltage = emon1.Vrms;  // Extract Vrms into a variable

  Serial.print("The RMS supply Voltage is: ");
  Serial.println(supplyVoltage);

  printMeasure("Irms1: ", currentRMS, "A");
  printMeasure("Irms2: ", currentRMS1, "A");
  delay(1000);

}

float getcurrent()        //to read value from 100A/1V
{ 
  float voltage;
  float current;
  float sum = 0;
  long time_check = millis();
  int counter = 0;

  while (millis() - time_check < 1000)
  {
    voltage = ads.readADC_Differential_0_1() * multiplier;
    current = voltage * FACTOR;
    //current /= 1000.0;

    sum += sq(current);
    counter = counter + 1;
  }

  current = sqrt(sum / counter);
  return (current);
}
float getcurrent1()       //to read value from CT 50A/1V
{
  float voltage1;
  float current1;
  float sum1 = 0;
  long time_check = millis();
  int counter = 0;

  while (millis() - time_check < 1000)
  {
    voltage1 = ads.readADC_Differential_2_3() * multiplier;
    current1 = voltage1 * FACTOR1;
    //current = 1000.0;

    sum1 += sq(current1);
    counter = counter + 1;
  }

  current1 = sqrt(sum1 / counter);
  return (current1);
}

