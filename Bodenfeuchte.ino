#define KDEBUG // comment this line to disable DEBUG mode
#ifdef KDEBUG
#include <DebugUtil.h>


// Get correct serial port for debugging
#ifdef __AVR_ATmega32U4__
// Leonardo/Micro/ProMicro use the USB serial port
#define DEBUGSERIAL Serial
#elif ESP8266
// ESP8266 use the 2nd serial port with TX only
#define DEBUGSERIAL Serial1
#else
// All other, (ATmega328P f.i.) use software serial
#include <SoftwareSerial.h>
SoftwareSerial softserial(11, 10); // RX, TX
#define DEBUGSERIAL softserial
#endif
// end of debugging defs
#endif

#include <KonnektingDevice.h>
#include <RunningMedian.h>
// Arduino pro Mini 3.3V 8MHZ
// "Vinduino" portable soil moisture sensor code V3.00
// Date December 31, 2012
// Reinier van der Lee and Theodore Kaskalis
// www.vanderleevineyard.com

// include the library code only for LCD display version
#include <math.h>
#include "kdevice_Bodenfeuchte.h"

// ################################################
// ### IO Configuration
// ################################################
#define NUM_READS 20    // Number of sensor reads for filtering
const long FirstKnown = 2998;  // Constant value of known resistor in Ohms
const long SecondKnown = 3000;  // Constant value of known resistor in Ohms
#define PROG_LED_PIN 13
#define PROG_BUTTON_PIN 8
#define SENSOR_PIN_1 6
#define SENSOR_PIN_2 7


// ################################################
// ### KONNEKTING Configuration
// ################################################
#ifdef __AVR_ATmega328P__
#define KNX_SERIAL Serial // Nano/ProMini etc. use Serial
#elif ESP8266
#define KNX_SERIAL Serial // ESP8266 use Serial
#else
#define KNX_SERIAL Serial1 // Leonardo/Micro etc. use Serial1
#endif





RunningMedian samples = RunningMedian(NUM_READS*2);


long knownResistor;
int activeDigitalPin = SENSOR_PIN_1;         // SENSOR_PIN_1 or SENSOR_PIN_2 interchangeably
int supplyVoltageAnalogPin;       // SENSOR_PIN_1-ON: A0, SENSOR_PIN_2-ON: A1
int sensorVoltageAnalogPin;       // SENSOR_PIN_1-ON: A1, SENSOR_PIN_2-ON: A0

float avr;
float moisture;

int supplyVoltage;                // Measured supply voltage
int sensorVoltage;                // Measured sensor voltage

float r[NUM_READS*2];

int i;                            // Simple index variable


//KNX Related
int16_t limitMoistMin;
int16_t limitMoistMax;
uint8_t valueMoistMin;
uint8_t valueMoistMax;

void setup() {
    // debug related stuff
#ifdef KDEBUG
    // Start debug serial with 115200 bauds
    DEBUGSERIAL.begin(9600);

  #ifdef __AVR_ATmega32U4__
      // wait for serial port to connect. Needed for Leonardo/Micro/ProMicro only
      while (!DEBUGSERIAL)
  #endif

    // make debug serial port known to debug class
    // Means: KONNEKTING will sue the same serial port for console debugging
    Debug.setPrintStream(&DEBUGSERIAL);
    Debug.println("Start");
#endif

  // initialize the digital pin as an output.
  // Pin SENSOR_PIN_1 is sense resistor voltage supply 1
  pinMode(SENSOR_PIN_1, OUTPUT);    

  // initialize the digital pin as an output.
  // Pin SENSOR_PIN_2 is sense resistor voltage supply 2
  pinMode(SENSOR_PIN_2, OUTPUT);   


  // Initialize KNX enabled Arduino Board
  Konnekting.init(KNX_SERIAL, PROG_BUTTON_PIN, PROG_LED_PIN, MANUFACTURER_ID, DEVICE_ID, REVISION);
  if (!Konnekting.isFactorySetting()){
         delay(3);
      }
     //get parameters here:  e.g. valueHumdMin = Konnekting.getUINT8Param(PARAM_rhMinValue);
     limitMoistMin = Konnekting.getINT16Param(PARAM_LowerAlarm);
     limitMoistMax = Konnekting.getINT16Param(PARAM_UpperAlarm);
     valueMoistMin = Konnekting.getUINT8Param(PARAM_SendOnLowerAlarm);
     valueMoistMax = Konnekting.getUINT8Param(PARAM_SendOnUpperAlarm);
  }




void limitReached(float comVal, float comValMin, float comValMax, int minObj, int maxObj, int minVal, int maxVal) {
    if (minVal != 255) {
        if (comVal <= comValMin) {
            Knx.write(minObj, minVal);
        }
    }
    if (maxVal != 255) {
        if (comVal >= comValMax) {
            Knx.write(maxObj, maxVal);
        }
    }
};

void loop() {
  Knx.task();
  // read sensor, filter, and calculate resistance value
  // Noise filter: median filter
  samples.clear();
  for (i=0; i<NUM_READS*2; i++) {

    setupCurrentPath();      // Prepare the digital and analog pin values

    // Read 1 pair of voltage values
    digitalWrite(activeDigitalPin, HIGH);                 // set the voltage supply on
    delay(10);
    supplyVoltage = analogRead(supplyVoltageAnalogPin);   // read the supply voltage
    sensorVoltage = analogRead(sensorVoltageAnalogPin);   // read the sensor voltage
    digitalWrite(activeDigitalPin, LOW);                  // set the voltage supply off  
    delay(100); 

    // Calculate resistance and moisture percentage without overshooting 100
    // the 0.5 add-term is used to round to the nearest integer
    // Tip: no need to transform 0-1023 voltage value to 0-5 range, due to following fraction
    //valueOf[i].resistance = long( float(knownResistor) * ( supplyVoltage - sensorVoltage ) / sensorVoltage + 0.5 );
    r[i]                  = long( float(knownResistor) * ( supplyVoltage - sensorVoltage ) / sensorVoltage + 0.5 );
    samples.add(r[i]);
    //DEBUG.print("R");
    //Debug.println(r[i]);
    //valueOf[i].moisture = min( int( pow( valueOf[i].resistance/31.65 , 1.0/-1.695 ) * 400 + 0.5 ) , 100 );
   // m[i]                = min( int( pow( r[i]/31.65 , 1.0/-1.695 ) * 400 + 0.5 ) , 100 );
   // m[i]                = ((4.093+3.213*r[i]/1000)/(1-0.009733*r[i]/1000-0.01205*10));  //10 is Assumed Soil Temp
//  valueOf[i].moisture = min( int( pow( valueOf[i].resistance/331.55 , 1.0/-1.695 ) * 100 + 0.5 ) , 100 );

  }

  // end of multiple read loop

  // Sort the moisture-resistance vector according to moisture
  //sortMoistures();

  // calculate average
  //avr=average (r, NUM_READS*2);
  avr=samples.getAverage(5);
  moisture=((4.093+3.213*avr/1000)/(1-0.009733*avr/1000-0.01205*10));
  
  // Print out values
  //DEBUG.print(sensorVoltage);
  //DEBUG.print("\t");
  //DEBUG.print(supplyVoltage);
  //DEBUG.print("\t");
  //DEBUG.print(moisture);
  //DEBUG.print("\t");
  //DEBUG.print(avr);
  //DEBUG.print("\t");
  //DEBUG.print(samples.getAverage());
  //DEBUG.print("\t");
  //DEBUG.print();
  //DEBUG.print("\t");
  #ifdef KDEBUG  
     // Debug.println(moisture);
  #endif
  
  if (Konnekting.isReadyForApplication()) {
    Knx.write(COMOBJ_moisture, moisture);
    Knx.write(COMOBJ_Resistance, avr);
    limitReached(moisture, limitMoistMin, limitMoistMax, COMOBJ_LowerAlarm, COMOBJ_UpperAlarm, valueMoistMin, valueMoistMax);  //hardcoded to send a 1 if exceeded
  }
  
  delay(5000);
     

}

void setupCurrentPath() {
  if ( activeDigitalPin == SENSOR_PIN_1 ) {
    activeDigitalPin = SENSOR_PIN_2;
    knownResistor   = FirstKnown;
    supplyVoltageAnalogPin = A1;
    sensorVoltageAnalogPin = A0;
  }
  else {
    activeDigitalPin = SENSOR_PIN_1;
    knownResistor   = SecondKnown;
    supplyVoltageAnalogPin = A0;
    sensorVoltageAnalogPin = A1;
  }
}

void knxEvents(byte index) {
    // nothing to do in this sketch
};


float average (float * array, int len)  // assuming array is int.
{
  float sum = 0 ;  // sum will be larger than an item, long for safety.
  //Debug.println("Averaging");
  for (int i = 0 ; i < len ; i++){
    //DEBUG.print(" "); 
    //DEBUG.print(array[i]);
    sum = sum+ array[i] ;
    //DEBUG.print(" Sum: ");
    //Debug.println(sum);
  }
  //DEBUG.print("Average: ");
  //Debug.println(((float) sum) / len);
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}
