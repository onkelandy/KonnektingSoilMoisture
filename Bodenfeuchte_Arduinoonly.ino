// "Vinduino-H" portable soil moisture sensor code V3.10
// Date November 16, 2014
// Copyright (C), 2015, Reinier van der Lee and Theodore Kaskalis
// www.vanderleevineyard.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

#include <math.h>
#include <RunningMedian.h>

#define NUM_READS 20    // Number of sensor reads for filtering

RunningMedian moisture_samples = RunningMedian(NUM_READS*2);
RunningMedian resistance_samples = RunningMedian(NUM_READS*2);
RunningMedian tension_samples = RunningMedian(NUM_READS*2);

const long knownResistor = 4700;  // Constant value of known resistor in Ohms
#define SENSOR_PIN_1 9
#define SENSOR_PIN_2 8

int activeDigitalPin = SENSOR_PIN_1;         // 6 or 7 interchangeably
int supplyVoltageAnalogPin;       // 6-ON: A0, 7-ON: A1
int sensorVoltageAnalogPin;       // 6-ON: A1, 7-ON: A0

int supplyVoltage;                // Measured supply voltage
int sensorVoltage;                // Measured sensor voltage

int RXLED = 17;

uint16_t delay_sensor = 5000;
uint16_t delay_on = 10;
uint16_t delay_off = 100;
uint32_t currentTime = 0;
uint32_t initTime = 0;
uint32_t delayTime = 0;
int runthroughs = -1;
uint8_t wrote = 0;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 

  // initialize the digital pin as an output.
  // Pin 6 is sense resistor voltage supply 1
  pinMode(SENSOR_PIN_1, OUTPUT);    

  // initialize the digital pin as an output.
  // Pin 7 is sense resistor voltage supply 2
  pinMode(SENSOR_PIN_2, OUTPUT);   
  
}

void loop() {
  //delay(3000);
  currentTime = millis();
  if (runthroughs < 0) {
      initTime = millis();
      runthroughs += 1;
      digitalWrite(SENSOR_PIN_1, LOW);
      digitalWrite(SENSOR_PIN_2, LOW);
      TXLED0;
      digitalWrite(RXLED, LOW);
    }
  if (currentTime - initTime > delay_sensor) {
    if (runthroughs == 0) {     
      tension_samples.clear();
      moisture_samples.clear();
      resistance_samples.clear();
      runthroughs += 1;
    }
    if (runthroughs < NUM_READS) {
      
      if (wrote == 0) {
        delayTime = millis();
        wrote += 1;
        
      }
      int timeDifference = currentTime - delayTime;
      if (timeDifference > delay_off && wrote == 1) {        
        digitalWrite(activeDigitalPin, HIGH);
        if (activeDigitalPin == SENSOR_PIN_1) {
          TXLED1;
        }
        else {
          digitalWrite(RXLED, HIGH);
        }
        wrote += 1;
        delayTime = millis(); 
      }
      
      timeDifference = currentTime - delayTime;
      if (timeDifference > delay_on && wrote == 2) {
        supplyVoltage = analogRead(supplyVoltageAnalogPin);   // read the supply voltage
        sensorVoltage = analogRead(sensorVoltageAnalogPin);   // read the sensor voltage
        //sensorVoltage = random(770,800); // for testing purposes only
        
        digitalWrite(activeDigitalPin, LOW);
        if (activeDigitalPin == SENSOR_PIN_1) {
          TXLED0;
        }
        else {
          digitalWrite(RXLED, LOW);
        }
        setupCurrentPath();
        calculate(supplyVoltage, sensorVoltage);
        runthroughs += 1;       
        wrote = 0;
      }
    }
    else if (runthroughs >= NUM_READS) {
      float resistance = resistance_samples.getMedian();
      float moisture = moisture_samples.getMedian();
      float tension = tension_samples.getMedian();
      Serial.print("Moisture: ");
      Serial.println(moisture);
      Serial.print("Tension: ");
      Serial.println(tension);
      Serial.print("resistance: ");
      Serial.println(resistance);
      runthroughs = -1;
      wrote = 0;
    }
  }
}

void calculate(int supplyVoltage, int sensorVoltage) {    
    // Calculate resistance and moisture percentage without overshooting 100
    // the 0.5 add-term is used to round to the nearest integer
    // Tip: no need to transform 0-1023 voltage value to 0-5 range, due to following fraction
    //Serial.print("Supply: ");
      //Serial.println(supplyVoltage);
      //Serial.print("Sensor: ");
      //Serial.println(sensorVoltage);
    float temp_resistance = long( float(knownResistor) * ( supplyVoltage - sensorVoltage ) / sensorVoltage + 0.5 );
    float temp_moisture = min( int( pow( temp_resistance/31.65 , 1.0/-1.695 ) * 400 + 0.5 ) , 100 );
    //float temp_moisture = ((4.093+3.213*temp_resistance/1000)/(1-0.009733*temp_resistance/1000-0.01205*10));
    float temp_tension = round( ( -3.213 * (double(temp_resistance)/1000) - 4.093) / ( 1 - 0.009733 * (double(temp_resistance)/1000) - 0.01205*24)); // Watermark 1 < 8 kOhm
      Serial.print("temp Moisture: ");
      Serial.println(temp_moisture);
      //Serial.print("temp resistance: ");
      //Serial.println(temp_resistance);
    resistance_samples.add(temp_resistance); 
    moisture_samples.add(temp_moisture); 
    tension_samples.add(temp_tension); 
}

void setupCurrentPath() {
  if ( activeDigitalPin == SENSOR_PIN_1 ) {
    activeDigitalPin = SENSOR_PIN_2;
    supplyVoltageAnalogPin = A1;
    sensorVoltageAnalogPin = A0;
  }
  else {
    activeDigitalPin = SENSOR_PIN_1;
    supplyVoltageAnalogPin = A0;
    sensorVoltageAnalogPin = A1;
  }
}
