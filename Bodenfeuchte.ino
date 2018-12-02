#include <KonnektingDevice.h>

// include device related configuration code, created by "KONNEKTING CodeGenerator"
#include "kdevice_Bodenfeuchte.h"

#include <RunningMedian.h>
#include <math.h>

// ################################################
// ### DEBUG CONFIGURATION
// ################################################
// #define KDEBUG // comment this line to disable DEBUG mode
#ifdef KDEBUG
  #include <DebugUtil.h>

  // Get correct serial port for debugging
  #ifdef __AVR_ATmega32U4__
    // Leonardo/Micro/ProMicro use the USB serial port
    #define DEBUGSERIAL Serial
  #elif ESP8266
    // ESP8266 use the 2nd serial port with TX only
    #define DEBUGSERIAL Serial1
  #elif __SAMD21G18A__
    #define DEBUGSERIAL SerialUSB
  #else
    // All other, (ATmega328P f.i.) use software serial
    #include <SoftwareSerial.h>
    SoftwareSerial softserial(11, 10); // RX, TX
    #define DEBUGSERIAL softserial
  #endif
  // end of debugging defs
#endif

// ################################################
// ### IO Configuration
// ################################################

const long knownResistor = 4700;
#define PROG_LED_PIN 8
#define PROG_BUTTON_PIN 7
#define SENSOR_PIN_1 9
#define SENSOR_PIN_2 8

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

uint8_t NUM_READS = 20;    // Number of sensor reads for filtering
RunningMedian moisture_samples = RunningMedian(NUM_READS);
RunningMedian resistance_samples = RunningMedian(NUM_READS);
RunningMedian tension_samples = RunningMedian(NUM_READS);

uint8_t activeDigitalPin = SENSOR_PIN_1;         // 6 or 7 interchangeably
uint8_t supplyVoltageAnalogPin;       // 6-ON: A0, 7-ON: A1
uint8_t sensorVoltageAnalogPin;       // 6-ON: A1, 7-ON: A0

uint16_t supplyVoltage;                // Measured supply voltage
uint16_t sensorVoltage;                // Measured sensor voltage

uint16_t delay_sensor = 5000; // standard delay for cycle, changed later
uint8_t delay_on = 20;
uint8_t delay_off = 100;
uint32_t currentTime = 0;
uint32_t initTime = 0;
uint32_t cycleTime = 0;
uint32_t delayTime = 0;
int runthroughs = -2;
uint8_t wrote = 0;

//KNX Related
uint8_t limitMoistMin;
uint8_t limitMoistMax;
uint8_t sendMoistMin;
uint8_t sendMoistMax;
bool repeatMoistMin = false;
bool repeatMoistMax = false;
bool cyclic = false;
uint8_t sendonchange;
uint32_t cycle;
float deltaMoisture = 1;

float resistance = 0.0;
float moisture = 0.0;
float tension = 0.0;
float prev_resistance = 0.0;
float prev_moisture = 0.0;
float prev_tension = 0.0;

// Callback function to handle com objects updates

void knxEvents(byte index) {
    // nothing to do in this sketch
};

void setup() {
    // debug related stuff
    #ifdef KDEBUG
        // Start debug serial with 115200 bauds
        DEBUGSERIAL.begin(9600);
        #if defined(__AVR_ATmega32U4__) || defined(__SAMD21G18A__)
            // wait for serial port to connect. Needed for Leonardo/Micro/ProMicro only
            while (!DEBUGSERIAL)
        #endif
        // make debug serial port known to debug class
        // Means: KONNEKTING will sue the same serial port for console debugging
        Debug.setPrintStream(&DEBUGSERIAL);
    #endif
    // Initialize KNX enabled Arduino Board
    Konnekting.init(KNX_SERIAL, PROG_BUTTON_PIN, PROG_LED_PIN, MANUFACTURER_ID, DEVICE_ID, REVISION);

    if (!Konnekting.isFactorySetting()){
        uint8_t startDelay = (int) Konnekting.getUINT8Param(PARAM_startup_delay) * 1000;
        if (startDelay > 0) {
            #ifdef KDEBUG
                 Debug.println(F("delay for %d ms"),startDelay);
            #endif
            delay(startDelay);
            #ifdef KDEBUG
                 Debug.println(F("ready!"));
            #endif
        }

        NUM_READS = (int) Konnekting.getINT16Param(PARAM_Samples);

        delay_sensor = (int) Konnekting.getINT8Param(PARAM_sensor_delay) * 1000;
        cycle = (int) Konnekting.getINT32Param(PARAM_cycle) * 1000;
        cyclic = (bool) Konnekting.getINT8Param(PARAM_cyclic);
        sendonchange = (int) Konnekting.getINT8Param(PARAM_SendOnChange);
        deltaMoisture = (int) Konnekting.getINT8Param(PARAM_DeltaMoisture) / 10;

        limitMoistMin = Konnekting.getINT8Param(PARAM_LowerAlarm);
        limitMoistMax = Konnekting.getINT8Param(PARAM_UpperAlarm);
        sendMoistMin = Konnekting.getUINT8Param(PARAM_SendOnLowerAlarm);
        sendMoistMax = Konnekting.getUINT8Param(PARAM_SendOnUpperAlarm);
        repeatMoistMin = (bool) Konnekting.getUINT8Param(PARAM_RepeatLowerAlarm);
        repeatMoistMax = (bool) Konnekting.getUINT8Param(PARAM_RepeatUpperAlarm);
    }

    #ifdef KDEBUG
        Debug.println(F("Cleared samples: %d"), moisture_samples.getCount());
        Debug.println(F("NUM READSs: %d"), NUM_READS);
        Debug.println(F("cyclic: %s"), cyclic ? "true" : "false");
        Debug.println(F("cycle: %d"), cycle);
        Debug.println(F("sendonchange: %d"), sendonchange);
        Debug.println(F("deltaMoisture: %d.%d"), int(deltaMoisture * 10.0)/10, int(deltaMoisture * 10.0)%10);
        Debug.println(F("limitMoistMin: %d"), limitMoistMin);
        Debug.println(F("limitMoistMax: %d"), limitMoistMax);
        Debug.println(F("sendMoistMin: %d"), sendMoistMin);
        Debug.println(F("sendMoistMax: %d"), sendMoistMax);
        Debug.println(F("repeatMoistMin: %s"), repeatMoistMin ? "true" : "false");
        Debug.println(F("repeatMoistMax: %s"), repeatMoistMax ? "true" : "false");
        Debug.println(F("Setup is ready. go to loop..."));
    #endif

}

void loop() {
    Knx.task();
    currentTime = millis();

    // only do measurements and other sketch related stuff if not in programming mode & not with factory settings
    if (Konnekting.isReadyForApplication()) {

      if (runthroughs < -1) {
          initTime = millis();
          runthroughs += 1;
          digitalWrite(SENSOR_PIN_1, LOW);
          digitalWrite(SENSOR_PIN_2, LOW);
      }
      if (currentTime - initTime >= delay_sensor) {
        if (runthroughs == -1) {
          tension_samples.clear();
          moisture_samples.clear();
          resistance_samples.clear();
          runthroughs += 1;
          #ifdef KDEBUG
              Debug.println(F("Cleared sample count: %d"), moisture_samples.getCount());
          #endif
        }
        if (runthroughs < NUM_READS) {
            if (wrote == 0) {
              delayTime = millis();
              wrote += 1;
            }
            if (currentTime - delayTime >= delay_off && wrote == 1) {
              digitalWrite(activeDigitalPin, HIGH);
              #ifdef KDEBUG
                  // Debug.println(F("currentTime: %ld. delayTime: %ld. Pin %d is high"), currentTime, delayTime, activeDigitalPin);
              #endif
              wrote += 1;
              delayTime = millis();
            }

            if (currentTime - delayTime >= delay_on / 2 && wrote == 2) {
              supplyVoltage = analogRead(supplyVoltageAnalogPin);   // read the supply voltage
              sensorVoltage = analogRead(sensorVoltageAnalogPin);   // read the sensor voltage
              // supplyVoltage = 750; // for testing purposes only
              sensorVoltage = random(520,540); // for testing purposes only
              #ifdef KDEBUG
                  //Debug.println(F("supplyVoltage: %d"), supplyVoltage);
                  //Debug.println(F("sensorVoltage: %d"), sensorVoltage);
              #endif
              calculate(supplyVoltage, sensorVoltage);
              wrote += 1;
            }
            if (currentTime - delayTime >= delay_on && wrote == 3) {
              runthroughs += 1;
              digitalWrite(activeDigitalPin, LOW);
              #ifdef KDEBUG
                  // Debug.println(F("currentTime: %ld. delayTime: %ld. Pin %d is low"), currentTime, delayTime, activeDigitalPin);
              #endif
              setupCurrentPath();
              wrote = 0;
            }
        }
        if (runthroughs >= NUM_READS) {
            #ifdef KDEBUG
                //Debug.println(F("currentTime: %ld"), currentTime);
                //Debug.println(F("cycleTime: %ld"), cycleTime);
                Debug.println(F("moisture after median: %d.%d"), int(moisture * 10.0)/10, int(moisture * 10.0)%10);
            #endif
            resistance = resistance_samples.getMedian();
            moisture = moisture_samples.getMedian();
            tension = abs(tension_samples.getMedian());
            limitReached(moisture, prev_moisture, limitMoistMin, limitMoistMax, COMOBJ_LowerAlarm, COMOBJ_UpperAlarm, sendMoistMin, sendMoistMax);
            runthroughs = -2;
            wrote = 0;
        }
      }

      if (currentTime - cycleTime > cycle && cyclic == 1) {
          cycleTime = millis();
          #ifdef KDEBUG
              Debug.println(F("cycleTime: %ld"), cycleTime);
          #endif
          writeValues(moisture, tension, resistance);
          prev_resistance = resistance;
          prev_moisture = moisture;
          prev_tension = tension;
          #ifdef KDEBUG
            Debug.println(F("cyclic moisture: %d.%d"), int(moisture * 10.0)/10, int(moisture * 10.0)%10);
          #endif
       }
      else if (abs(prev_moisture - moisture) >= deltaMoisture) {
          #ifdef KDEBUG
              Debug.println(F("changed moisture: %d.%d"), int(moisture * 10.0)/10, int(moisture * 10.0)%10);
          #endif
          if (sendonchange <= 1) {
              Knx.write(COMOBJ_Change, sendonchange);
              writeValues(moisture, tension, resistance);
              prev_resistance = resistance;
              prev_moisture = moisture;
              prev_tension = tension;
          }
       }
   }
}


void calculate(int supplyVoltage, int sensorVoltage) {

    float temp_resistance = long( float(knownResistor) * ( supplyVoltage - sensorVoltage ) / sensorVoltage + 0.5 );
    float temp_moisture = min( int( pow( temp_resistance/31.65 , 1.0/-1.695 ) * 400 + 0.5 ) , 100 );
    //float temp_moisture = ((4.093+3.213*temp_resistance/1000)/(1-0.009733*temp_resistance/1000-0.01205*10));
    float temp_tension = round( ( -3.213 * (double(temp_resistance)/1000) - 4.093) / ( 1 - 0.009733 * (double(temp_resistance)/1000) - 0.01205*24)); // Watermark 1 < 8 kOhm
    resistance_samples.add(temp_resistance);
    moisture_samples.add(temp_moisture);
    tension_samples.add(temp_tension);
    #ifdef KDEBUG
        Debug.println(F("resistance: %d.%d"), int(temp_resistance * 10.0)/10, int(temp_resistance * 10.0)%10);
        Debug.println(F("moisture: %d.%d"), int(temp_moisture * 10.0)/10, int(temp_moisture * 10.0)%10);
        //Debug.println(F("moisture median: %d.%d"), int(moisture_samples.getMedian() * 10.0)/10, int(moisture_samples.getMedian() * 10.0)%10);
        Debug.println(F("tension: %d.%d"), int(temp_tension * 10.0)/10, int(temp_tension * 10.0)%10);
        //Debug.println(F("Sample count: %d"), moisture_samples.getCount());
    #endif
}

void writeValues(float moisture, float tension, float resistance) {
    Knx.write(COMOBJ_Moisture, moisture);
    Knx.write(COMOBJ_Tension, tension);
    Knx.write(COMOBJ_Resistance, resistance);
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

void limitReached(float comVal, float prevVal, float comValMin, float comValMax, int minObj, int maxObj, int minVal, int maxVal) {
    if (minVal != 255) {
        if ((comVal <= comValMin && (prevVal > comValMin || prevVal == 0)) || (repeatMoistMin && comVal <= comValMin && prevVal <= comValMin && abs(comVal - prevVal) >= deltaMoisture)) {
            Knx.write(minObj, minVal);
            #ifdef KDEBUG
                Debug.println(F("moisture below lower threshold: %d.%d"), int(comVal * 10.0)/10, int(comVal * 10.0)%10);
            #endif
        }
    }
    if (maxVal != 255) {
        if ((comVal >= comValMax && (prevVal < comValMax || prevVal == 0)) || (repeatMoistMax && comVal >= comValMax && prevVal >= comValMax && abs(comVal - prevVal) >= deltaMoisture)) {
            Knx.write(maxObj, maxVal);
            #ifdef KDEBUG
                Debug.println(F("moisture above upper threshold: %d.%d"), int(comVal * 10.0)/10, int(comVal * 10.0)%10);
            #endif
        }
    }
}
