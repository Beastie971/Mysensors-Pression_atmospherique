//#define MY_DEBUG 
#define MY_RADIO_RF24
#define MY_NODE_ID 4
#define DEBUG // Debug hors Mysensors 

// A 2.4Ghz transmitter and receiver, often used with MySensors.
// #define MY_RF24_PA_LEVEL RF24_PA_MIN              // This sets a low-power mode for the radio. Useful if you use the version with the bigger antenna, but don't want to power that from a separate power source. It can also fix problems with fake Chinese versions of the radio.

// Do you want this sensor to also be a repeater?
// #define MY_REPEATER_FEATURE 

// VARIABLES YOU CAN CHANGE
#define GENERATE_FORECAST
#define ALTITUDE 170 //current altitude in meters
#define SEALEVELPRESSURE_HPA 1013.25


#define DELAY 60000 // ms


#define COMPARE_TEMP 1                            // Send temperature only if it changed? 1 = Yes 0 = No. Can save battery.
float tempThreshold = 0.1;                        // How big a temperature difference has to minimally  be before an update is sent. Makes the sensor less precise, but also less jittery, and can save battery.
#define COMPARE_BARO 1                            // Send barometric pressure only if changed? 1 = Yes 0 = No. Can save battery.
float presThreshold = 0.1;
#define CONVERSION_FACTOR (1.0/10.0)     // used by forecast algorithm to convert from Pa to kPa, by dividing hPa by 10.


#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <MySensors.h>

#define BARO_CHILD_ID 0
#define TEMP_CHILD_ID 1
#define FORECAST_CHILD_ID 2

//PIN config
#define BMP_SCK 4
//#define BMP_MISO 12
#define BMP_MOSI 5
#define BMP_CS 10




Adafruit_BMP3XX bmp;
float Prel=0,lastPrel,Pabs;
float temperature,lastTemperature;

float T = 0;


#ifdef GENERATE_FORECAST             //  Below you will find a lot of variables used by the forecast algorithm.
const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
  STABLE = 0,                     // "Stable Weather Pattern"
  SUNNY = 1,                      // "Slowly rising Good Weather", "Clear/Sunny "
  CLOUDY = 2,                     // "Slowly falling L-Pressure ", "Cloudy/Rain "
  UNSTABLE = 3,                   // "Quickly rising H-Press",     "Not Stable"
  THUNDERSTORM = 4,                 // "Quickly falling L-Press",    "Thunderstorm"
  UNKNOWN = 5                     // "Unknown (More Time needed)
};
int lastForecast = -1;
int forecast = -1; // Stores the previous forecast, so it can be compared with a new forecast.
const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];
int minuteCount = 0;                // Helps the forecast algorithm keep time.
bool firstRound = true;                // Helps the forecast algorithm recognise if the sensor has just been powered up.
float pressureAvg;                // Average value is used in forecast algorithm.
float pressureAvg2;                // Average after 2 hours is used as reference value for the next iteration.
float dP_dt;                    // Pressure delta over time
#endif

// MYSENSORS COMMUNICATION VARIABLES
MyMessage temperatureMsg(TEMP_CHILD_ID, V_TEMP);
MyMessage pressureMsg(BARO_CHILD_ID, V_PRESSURE);
#ifdef GENERATE_FORECAST
MyMessage forecastMsg(BARO_CHILD_ID, V_FORECAST);
#endif
    
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");


  
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
    
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  bmp.performReading();
  sleep(1);
  
  }
}
  

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Pressure BMP388 Sensor", "3.4");

  // Tell the MySensors gateway what kind of sensors this node has, and what their ID's on the node are, as defined in the code above.
  present(BARO_CHILD_ID, S_BARO);
  present(TEMP_CHILD_ID, S_TEMP);
}


void loop() {

if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }



  T = bmp.temperature;
  Serial.print("Temperature = ");
  Serial.print(T);
  Serial.println(" °C");

  Serial.print("Pabs = ");
  Pabs = bmp.pressure / 100;
  
  Serial.print(Pabs);
  Serial.println(" hPa");
  
  //Prel = (Pabs + ALTITUDE /8.3);
  Prel =  (((Pabs * 100.0)/pow((1-((float)(ALTITUDE))/44330), 5.255))/100.0);

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m"); 


  Serial.print("Prel= ");
  Serial.print(Prel);
  Serial.println(" hPa");

  #ifdef GENERATE_FORECAST      
    int forecast = sample(Prel);            // Run the forecast function with a new pressure update. 
    Serial.print("Forecast = ");
    Serial.println(forecast); 
    if (forecast != lastForecast) {
      Serial.println("BMP380 - Sending the latest forecast to the gateway.");  
      /*if ( forecast == UNKNOWN and Prel >= 1013,25 ) {
          forecast = SUNNY;
      }
      else {
        forecast = CLOUDY;
      } */

      send(forecastMsg.set(weather[forecast]));
      lastForecast = forecast;
    }
  #endif      

    


  
  if(Prel > 806.93 ) {
    Serial.print("BMP380 - Sending the new pressure to the gateway: ");
    Serial.println(Prel);
    send(pressureMsg.set(Prel, 1));
  }
  lastPrel = Prel; // Save new pressure to be able to compare in the next round.

  Serial.print("BMP380 - Sending the new temperature to the gateway.\n");
  send(temperatureMsg.set(bmp.temperature, 1));
  lastTemperature = bmp.temperature; // Save new temperatures to be able to compare in the next round.
 
  
  Serial.println();
  sleep(DELAY);
          

}



#ifdef GENERATE_FORECAST
// These functions are only included if the forecast function is enables. The are used to generate a weater prediction by checking if the barometric pressure is rising or falling over time.

float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++) {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }
  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  return lastPressureSamplesAverage;
}


// Forecast algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure) {
  // Calculate the average of the last n minutes.
  int index = minuteCount % LAST_SAMPLES_COUNT;
  lastPressureSamples[index] = pressure;

  minuteCount++;
  if (minuteCount > 185) {
    minuteCount = 6;
  }

  if (minuteCount == 5) {
    pressureAvg = getLastPressureSamplesAverage();
  }
  else if (minuteCount == 35) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour 
      dP_dt = change * 2; // note this is for t = 0.5hour
    }
    else {
      dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
    }
  }
  else if (minuteCount == 65) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { //first time initial 3 hour
      dP_dt = change; //note this is for t = 1 hour
    }
    else {
      dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 95) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 1.5; // note this is for t = 1.5 hour
    }
    else {
      dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 125) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    pressureAvg2 = lastPressureAvg; // store for later use.
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 2; // note this is for t = 2 hour
    }
    else {
      dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 155) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 2.5; // note this is for t = 2.5 hour
    } 
    else {
      dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 185) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 3; // note this is for t = 3 hour
    } 
    else {
      dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
    }
    pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

 

  int forecast = UNKNOWN;
  if (minuteCount < 35 && firstRound) { //if time is less than 35 min on the first 3 hour interval.
    forecast = UNKNOWN;
  }
  else if (dP_dt < (-0.25)) {
    forecast = THUNDERSTORM;
  }
  else if (dP_dt > 0.25) {
    forecast = UNSTABLE;
  }
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05))) {
    forecast = CLOUDY;
  }
  else if ((dP_dt > 0.05) && (dP_dt < 0.25)) {
    forecast = SUNNY;
  }
  else if ((dP_dt >(-0.05)) && (dP_dt < 0.05)) {
    forecast = STABLE;
  }
  else {
    forecast = UNKNOWN;
  }

  // uncomment when debugging
   #ifdef DEBUG
    Serial.print(F("BME280 - Forecast at minute "));
    Serial.print(minuteCount);
    Serial.print(F(" dP/dt = "));
    Serial.print(dP_dt);
    Serial.print(F("kPa/h --> "));
    Serial.println(weather[forecast]);
   #endif

  return forecast;
}
#endif
