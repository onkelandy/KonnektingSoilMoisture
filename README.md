# KonnektingSoilMoisture
## Installation:
Add RunningMedian and Konnekting to your Arduino library

Uncomment line "#define KDEBUG" to activate debugging via Serial Monitor
Change the following lines appropriately:
* const long knownResistor = 4700;
* define PROG_LED_PIN 8
* define PROG_BUTTON_PIN 7
* define SENSOR_PIN_1 9
* define SENSOR_PIN_2 8

Upload Arduino Code on Konnekting Device

## Configuration:
Run Konnekting Suite and add bodenfeuchte Device. Set the physical address, group addresses and parameters as follows:

### Allgemein/General
* Startup Delay
* Pause between measurements
* Number sample measurements used for calculating Median

### Bodenfeuchte/Soil Moisture
* Send value periodically: no/yes
* Pause between periodic sending in seconds
* Send value on change: on/off/none
* Minimum change for sending value in percent
* Lower threshold in percent
* Value to send when below lower threshold: on/off/none
* Repeat send below lower threshold: repeat/only send once
* Upper threshold in percent
* Value to send when above upper threshold: on/off/none
* Repeat send above upper threshold: repeat/only send once
