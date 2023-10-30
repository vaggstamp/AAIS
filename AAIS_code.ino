/* Automatic Autonomous Irrigation System (AAIS)
  It uses a two axis solar tracker to charge the 3.7v Li-ion Battery that supplies the system.
  It measures the soil humidity, the water level of the water tank, the air temperature and humidity, the battery level and the state of the solar tracker.
  It waters the plant automatically.
  It sends these measurments to the Virtuino app in order to have an idea of what's going on.
  When the AAWS initially connects to the app you have about 30 secs to set the soil threshold and the waterpump duration depending on your conditions.

  Author: Evangelos Stampoulis */

#include <SoftwareSerial.h>
SoftwareSerial espSerial =  SoftwareSerial(2, 3);                // Communication between Arduino Uno and Esp01

//------------- WiFi SETTINGS --------------------------------------------------------------------------------------------------------

const char* ssid = "Your Wifi Name";                             // WiFi credentials     
const char* password = "Your WiFi Password";                        
int port = 8000;                                                 // Virtuino default Server port
const char* serverIP = "192.168.1.150";                          // The three first numbers have to be the same with the router IP

//------------- VirtuinoCM Library and settings --------------------------------------------------------------------------------------

#include "VirtuinoCM.h"
VirtuinoCM virtuino;
#define V_memory_count 18                                        // the size of V memory. You can change it to a number <=255)
float V[V_memory_count];                                         // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.
boolean debug = true;                                            // set this variable to false on the finale code to decrease the request time.

#define espChPdPin 4                                             // CH_PD pin of the ESP-01 connected to Arduino Uno pin 4

unsigned long previousWorkTime = 0;
unsigned long workInterval = 10 * 60000;

//------------- voltage reading ------------------------------------------------------------------------------------------------------

#define battery A4                                               // The battery reader is Arduino Uno pin 4

#define fullBat 4.2                                              // Percentage Calibration
#define lowBat 3.4

#define voltSample 20                                            // Sampling values
#define samplingRate 30
// global variables
float sumVolts = 0;
float batPercentage = 0;

//------------- tepmerature and humidity sensor --------------------------------------------------------------------------------------

#include <Adafruit_Sensor.h>                                     // Include the libraries
#include <DHT.h>

#define DHTPIN 8                                                 // Set DHT pin

#define DHTTYPE DHT11                                            // Set DHT type

DHT dht = DHT(DHTPIN, DHTTYPE);                                  // Initialize DHT sensor for normal 16mhz Arduino

float humid = 0;                                                 // Global variables
float temp = 0;
float f = 0;
float hif = 0;
float hic = 0;

//------------- solar tracker --------------------------------------------------------------------------------------------------------

#include <Servo.h>                                               // Include the library

#define upThreshold 30                                           // Thresholds and error
#define downThreshold 150
#define leftThreshold 0
#define rightThreshold 180
int errorValue = 50;

#define night 700                                                // Night value (real night value = 400)

#define steptime 40                                              // Waittimes and steptime
unsigned long previousTrackingTime = -50000;
const unsigned long trackingInterval = 10 * 60000;
const unsigned long trackingTimeOut = 50000;

#define tdServoPower 5                                           // Servo pins 
#define lrServoPower 6
#define tdServoPWM 10
#define lrServoPWM 11

Servo tdServo;
Servo lrServo;

int trackingState = 0;                                           // Global variables
int upDownDegrees = 0;
int leftRightDegrees = 0;

//------------- waterlevel sensor,  soil sensor and waterpump ---------------------------------------------------------------------------

#define waterLevelSensorPower 13                                 // Sensor pins
#define soilSensorPower 12
#define waterPumpPower 7

#define emptyThreshold 70                                        // Thresholds
#define waterLowerThreshold 170
#define waterUpperThreshold 300

#define lowSoil 200                                              // Percentage calibration
#define fullSoil 1023

#define waterLevelSample 20                                      // Samples
#define soilSample 20
#define samplingRate 30

#define s0Pin 9                                                  // MUX pins
#define zPin A5

int waterLevel = 0;                                              // Global variables
int waterLevelState = -1;
int soilHumidity = 0;
int soilState = -1;
int waterPumpDuration = 5;
int soilThreshold = 50;

//============================================================== setup ===============================================================
//====================================================================================================================================
//====================================================================================================================================
//====================================================================================================================================

void setup() {
  if (debug) {
    Serial.begin(9600);
    while (!Serial) continue;
  }
//------------- voltage Reader -------------------------------------------------------------------------------------------------------

  pinMode(battery, INPUT);                                       // Input

  batteryRead();                                                 // Battery percentage Measuring

  delay(1000);

//------------- temperature and humidity sensor---------------------------------------------------------------------------------------

  dht.begin();                                                   // Setup sensor

  tempRead();                                                    // Temperature Measuring

  delay(1000);

//------------- solar tracker --------------------------------------------------------------------------------------------------------

  pinMode(A0, INPUT);                                            //Inputs
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  pinMode(lrServoPower, OUTPUT);                                 // Outputs
  pinMode(tdServoPower, OUTPUT);

  tdServo.attach(tdServoPWM);                                    // Attaching the servos to the 10 and 11 pins
  lrServo.attach(lrServoPWM);

  digitalWrite(lrServoPower, HIGH);                              // Giving power to the 2 servos
  digitalWrite(tdServoPower, HIGH);
  delay(100);

  tdServo.write(90);                                             // Telling servo to go to the starting position at top-down axis
  delay(1000);

  lrServo.write(90);                                             // Telling servo to go to the starting position at left-right axis
  delay(1000);

  trackingState = 1;                                             // Setting the values to the variables that are going to be sent
  upDownDegrees = tdServo.read();
  leftRightDegrees = lrServo.read();

  digitalWrite(lrServoPower, LOW);                               // Deactivate the servos for power saving
  digitalWrite(tdServoPower, LOW);

//------------- water level sensor,  soil sensor and waterpump -----------------------------------------------------------------------
  
  pinMode(zPin, INPUT);                                          // Input (Output of the mux)
  
  pinMode(s0Pin, OUTPUT);                                        // Output (Input of the mux)
  digitalWrite(s0Pin, LOW);                                      // MUX setup
  
  pinMode(waterLevelSensorPower, OUTPUT);                        // WaterLevelSensor setup
  digitalWrite(waterLevelSensorPower, LOW);
  
  pinMode(soilSensorPower, OUTPUT);                              // SoilSensorPower setup
  digitalWrite(soilSensorPower, LOW);
  
  pinMode(waterPumpPower, OUTPUT);                               // Water pump setup
  digitalWrite(waterPumpPower, LOW);
  
  waterLevel = readWaterLevelSensor() ;                          // Function that reads the water level value
  
  digitalWrite(s0Pin, HIGH);                                     // Changes the output of the mux to channel 1 (soil sensor)
  
  soilHumidity = readSoilSensor();                               // Function that reads the analog value from the soil sensor
  
  digitalWrite(s0Pin, LOW);                                      // Changes the output of the mux to channel 0 (water level sensor)
  
//------------- WiFi -----------------------------------------------------------------------------------------------------------------
  
  digitalWrite(espChPdPin, HIGH);                                // Activating the Esp01
  Serial.println("ESP On");
  
  espSerial.begin(9600);                                         // Setting the baud rate of espSerial
  espSerial.setTimeout(50);                                   
  
  virtuino.begin(onReceived,onRequested,256);                    // Start Virtuino. Set the buffer to 256. With this buffer Virtuino can control about 28 pins (1 command = 9bytes) The T(text) commands with 20 characters need 20+6 bytes
  //virtuino.key="1234";                                         // This is the Virtuino password. Only requests the start with this key are accepted from the library
  
  connectToWiFiNetwork();                                        // Connects to the WiFi Network
  
   vDelay(15000);
  
   if (V[9]!= soilThreshold && V[9]!= 0) {                       // The V9 has changed
       soilThreshold=V[9];                                       // Store the V9 to the variable soilThreshold so as to know if it has changed
   }
   if (V[8]!= waterPumpDuration && V[8]!= 0) {                   // The V8 has changed
       waterPumpDuration=V[8];                                   // Store the V8 to the variable waterPumpDuration so as to know if it has changed
   }
  
   if (soilHumidity <= 2.5){                                     // Change the soil state by the given soil threshold
      Serial.print("Soil sensor not attached (");
      soilState = -1;
   }
   else if (soilHumidity <= soilThreshold){
      Serial.print("The soil is DRY (");
      soilState = 0;
   }
   else {
      Serial.print("The soil is WET (");
      soilState = 1;
   }
  
   V[12] = waterPumpDuration;                                    // Sending the values to Virtuino App
   V[11] = soilThreshold;
   V[4] = batPercentage;
   V[5] = temp;
   V[3] = humid;
   V[2] = hic;
   V[7] = soilHumidity;
   V[1] = waterLevel;
   V[17] = waterLevelState;
   V[16] = soilState;
   V[15] = trackingState;
   V[13] = upDownDegrees;
   V[14] = leftRightDegrees;
 
   vDelay(15000);
  
   digitalWrite(espChPdPin, LOW);                                 // Deactivating the Esp01 for power saving
   Serial.println("ESP Off");
  
   waterPump();                                                   // Watering
}

//============================================================== loop ================================================================
//====================================================================================================================================
//====================================================================================================================================
//====================================================================================================================================

void loop() {

  unsigned long currentTime = millis();

  if ( currentTime - previousTrackingTime >= trackingInterval ) { // Tracking

      previousTrackingTime = currentTime;
      solarTracker();

  }

  if(currentTime - previousWorkTime >= workInterval){           // Measuring, Sending and Watering
  
      previousWorkTime = currentTime;
  
      batteryRead();                                            // Measuring
  
      vDelay(1000);
  
      tempRead();
  
      vDelay(1000);
  
      waterLevel = readWaterLevelSensor() ;
  
      digitalWrite(s0Pin, HIGH);
  
      soilHumidity = readSoilSensor();
  
      digitalWrite(s0Pin, LOW);
  
      vDelay(1000);
  
      digitalWrite(espChPdPin, HIGH);                               // Sending
      Serial.println("ESP On");
  
      virtuino.begin(onReceived,onRequested,256);
      //virtuino.key="1234";
  
      connectToWiFiNetwork();
  
      vDelay(15000);

      if (V[9]!= soilThreshold && V[9]!= 0) {                       // The V9 has changed
          soilThreshold=V[9];                                       // Store the V9 to the variable soilThreshold so as to know if it has changed
      }
      if (V[8]!= waterPumpDuration && V[8]!= 0) {                   // The V8 has changed
          waterPumpDuration=V[8];                                   // Store the V8 to the variable waterPumpDuration so as to know if it has changed
      }
  
      if (soilHumidity <= 2.5){                                     // Change the soil state by the given soil threshold
         Serial.print("Soil sensor not attached (");
         soilState = -1;
      }
      else if (soilHumidity <= soilThreshold){
         Serial.print("The soil is DRY (");
         soilState = 0;
      }
      else {
         Serial.print("The soil is WET (");
         soilState = 1;
      }
     
      V[12] = waterPumpDuration;
      V[11] = soilThreshold;
      V[4] = batPercentage;
      V[5] = temp;
      V[3] = humid;
      V[2] = hic;
      V[7] = soilHumidity;
      V[1] = waterLevel;
      V[17] = waterLevelState;
      V[16] = soilState;
      V[15] = trackingState;
      V[13] = upDownDegrees;
      V[14] = leftRightDegrees;
  
      vDelay(15000);
  
      digitalWrite(espChPdPin, LOW);
      Serial.println("ESP Off");
  
      waterPump();                                                // Watering
      }
}

//=========================================================== functions ==============================================================
//====================================================================================================================================
//====================================================================================================================================
//====================================================================================================================================



//------------- connectToWiFiNetwork -------------------------------------------------------------------------------------------------

void connectToWiFiNetwork() {
  Serial.println("Connecting to " + String(ssid));
  while (espSerial.available()) espSerial.read();
  espSerial.println("AT+GMR");                                // print firmware info
  waitForResponse("OK", 1000);
  espSerial.println("AT+CWMODE=1");                           // configure as client
  waitForResponse("OK", 1000);
  espSerial.print("AT+CWJAP=\"");                             // connect to your WiFi network
  espSerial.print(ssid);
  espSerial.print("\",\"");
  espSerial.print(password);
  espSerial.println("\"");
  waitForResponse("OK", 10000);
  espSerial.print("AT+CIPSTA=\"");                            // set IP
  espSerial.print(serverIP);
  espSerial.println("\"");
  waitForResponse("OK", 5000);
  espSerial.println("AT+CIPSTA?");
  waitForResponse("OK", 3000);
  espSerial.println("AT+CIFSR");                              // get ip address
  waitForResponse("OK", 1000);
  espSerial.println("AT+CIPMUX=1");                           // configure for multiple connections
  waitForResponse("OK", 1000);
  espSerial.print("AT+CIPSERVER=1,");
  espSerial.println(port);
  waitForResponse("OK", 1000);
}

//------------- On Received ---------------------------------------------------------------------------------------------------------

/* This function is called every time Virtuino app sends a request to server to change a Pin value
  The 'variableIndex' is the pin number index of Virtuino app
  The 'valueAsText' is the value that has sent from the app  */
void onReceived(char variableType, uint8_t variableIndex, String valueAsText) {
  if (variableType == 'V') {
    if (variableIndex < V_memory_count) {
      float value = valueAsText.toFloat();
      V[variableIndex] = value;
    }
  }
}

//------------- On Requested --------------------------------------------------------------------------------------------------------
/* This function is called every time Virtuino app requests to read a pin value*/
String onRequested(char variableType, uint8_t variableIndex) {
  if (variableType == 'V') {
    if (variableIndex < V_memory_count) return  String(V[variableIndex]); // return the value of the arduino V memory array
  }
  return "";
}

//------------- Virtuino Run --------------------------------------------------------------------------------------------------------
void virtuinoRun() {
  if (espSerial.available()) {
    virtuino.readBuffer = espSerial.readStringUntil('\n');
    if (debug) Serial.print('\n' + virtuino.readBuffer);
    int pos = virtuino.readBuffer.indexOf("+IPD,");
    if (pos != -1) {
      int connectionId = virtuino.readBuffer.charAt(pos + 5) - 48; // get connection ID
      int startVirtuinoCommandPos = 1 + virtuino.readBuffer.indexOf(":");
      virtuino.readBuffer.remove(0, startVirtuinoCommandPos);
      String* response = virtuino.getResponse();   // get the text that has to be sent to Virtuino as reply. The library will check the inptuBuffer and it will create the response text
      if (debug) Serial.println("\nResponse : " + *response);
      if (response->length() > 0) {
        String cipSend = "AT+CIPSEND=";
        cipSend += connectionId;
        cipSend += ",";
        cipSend += response->length();
        cipSend += "\r\n";
        while (espSerial.available()) espSerial.read();   // clear espSerial buffer
        for (int i = 0; i < cipSend.length(); i++) espSerial.write(cipSend.charAt(i));
        if (waitForResponse(">", 1000)) espSerial.print(*response);
        waitForResponse("OK", 1000);
      }
      espSerial.print("AT+CIPCLOSE="); espSerial.println(connectionId);
    }// (pos!=-1)

  } // if espSerial.available

}

//=================================================== waitForResponse
boolean waitForResponse(String target1,  int timeout) {
  String data = "";
  char a;
  unsigned long startTime = millis();
  boolean rValue = false;
  while (millis() - startTime < timeout) {
    while (espSerial.available() > 0) {
      a = espSerial.read();
      if (debug) Serial.print(a);
      if (a == '\0') continue;
      data += a;
    }
    if (data.indexOf(target1) != -1) {
      rValue = true;
      break;
    }
  }
  return rValue;
}

//------------- vDelay ---------------------------------------------------------------------------------------------------------------

void vDelay(int delayInMillis) {
  long t = millis() + delayInMillis;
  while (millis() < t) virtuinoRun();
}

//------------- Voltage Read ---------------------------------------------------------------------------------------------------------

void batteryRead() {
  const float voltsc = 0.00497;
  float volts[voltSample];
  float counts[voltSample];
  sumVolts = 0;

  unsigned long startTime = millis();
  unsigned int sampleCount = 0;

  while (sampleCount < voltSample) {
    if (millis() - startTime >= samplingRate) {
      counts[sampleCount] = analogRead(battery);
      volts[sampleCount] = counts[sampleCount] * voltsc;
      startTime = millis();
      sampleCount++;
    }
  }

  for (int j = 0; j < voltSample; j++) {
    sumVolts += volts[j];
  }

  if((((sumVolts / voltSample - lowBat) / (fullBat - lowBat)) * 100) > 100){
    batPercentage = 100;
  }
  else if((((sumVolts / voltSample - lowBat) / (fullBat - lowBat)) * 100) < 0){
    batPercentage = 0;
  }
  else{
    batPercentage = ((sumVolts / voltSample - lowBat) / (fullBat - lowBat)) * 100;
  }
  
  Serial.print("Battery: ");
  Serial.print(batPercentage);
  Serial.println("%");
  return;
}

//------------- Temperature Read -----------------------------------------------------------------------------------------------------

void tempRead() {

  humid = dht.readHumidity();                                  // Reading the humidity in %

  temp = dht.readTemperature();                                // Reading the temperature as Celsius

  f = dht.readTemperature(true);                               // Reading the temperature as Fahrenheit

  if (isnan(humid) || isnan(temp) || isnan(f)) {               // Checking if any reads failed and exit early (to try again)
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  hif = dht.computeHeatIndex(f, humid);                        // Computing heat index in Fahrenheit (default):

  hic = dht.computeHeatIndex(temp, humid, false);              // Computing heat index in Celsius

  Serial.print("Humidity: ");                                  // Printing the values
  Serial.print(humid);
  Serial.print(" % ");
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print(" \xC2\xB0");
  Serial.print("C | ");
  Serial.print(hic);
  Serial.print(" \xC2\xB0");
  Serial.println("C");
}

//------------- Solar Tracking ---------------------------------------------------------------------------------------------------------

void solarTracker() {

  unsigned long previousTime = millis();

  digitalWrite(lrServoPower, HIGH);                             // Giving power to the 2 servos
  digitalWrite(tdServoPower, HIGH);

  while (millis() - previousTime <= trackingTimeOut) {          // Trying to track successfully for trackingTimeOut milliseconds

    int topleft = analogRead(A0);                               // Reading the photoresistors
    int topright = analogRead(A3);
    int downleft = analogRead(A1);
    int downright = analogRead(A2);

    int avgres = (topleft + topright + downleft + downright) / 4; // Average value of all the photoresistors

    if (avgres <= 50) {                                         // Changing the error value depending the brightness of the sun at every step
      errorValue = 5;
    }
    else if (avgres <= 100) {
      errorValue = 10;
    }
    else if (avgres <= 200) {
      errorValue = 30;
    }
    else if (avgres <= 400) {
      errorValue = 80;
    }
    else if (avgres <= 600) {
      errorValue = 100;
    }
    else errorValue = 200;

    vDelay(100);

    if (topleft - topright < errorValue && topleft - topright > -errorValue && downleft - downright < errorValue && downleft - downright > -errorValue && topleft - downleft < errorValue && topleft - downleft > -errorValue && topright - downright < errorValue && topright - downright > -errorValue) {

//      Serial.print("Centralized\n");                           // If all the values are close enough (errorValue)
//      Serial.print("topleft:");
//      Serial.println(topleft);
//      Serial.print("topright:");
//      Serial.println(topright);
//      Serial.print("downleft:");
//      Serial.println(downleft);
//      Serial.print("downright:");
//      Serial.println(downright);
//      Serial.print("avgres:");
//      Serial.println(avgres);
//      Serial.print("error:");
//      Serial.println(errorValue);
//      Serial.println();

      if (avgres > night) {                                    // If it is dark outside

        tdServo.write(90);                                     // Telling servo to go to the starting position waiting for the dawn at top-down axis

        vDelay(1000);

        lrServo.write(90);                                     // Telling servo to go to the starting position waiting for the dawn at left-right axis

        vDelay(1000);

        trackingState = 0;                                     // Changing the variable that will be sent
        upDownDegrees = tdServo.read();
        leftRightDegrees = lrServo.read();

        digitalWrite(lrServoPower, LOW);                       // Zeroing the standby current of the servos until next tracking
        digitalWrite(tdServoPower, LOW);
        return;                                                // Staying there and wait till morning
      }

      else {                                                   // If it is bright outside

        trackingState = 1;                                     // Changing the variable that will be sent
        upDownDegrees = tdServo.read();
        leftRightDegrees = lrServo.read();

        digitalWrite(lrServoPower, LOW);                       // Zeroing the standby current of the servos until next tracking
        digitalWrite(tdServoPower, LOW);

        return;                                                // Staying there to absorb the sunlight
      }
    }

    if (tdServo.read() > 90) {                                  // If the panel is 90 degrees and above at the top-down axis (not centeralized)

      if (topleft - topright < errorValue && topleft - topright > -errorValue) { // If topleft and topright values are about the same, do nothing
        vDelay(steptime);
      }
      else if (topleft > topright) {                            // If topleft value is much darker than topright
        lrServo.write(lrServo.read() + 1);                      // Go to the right by 1 degree
        vDelay(steptime);
      }
      else {                                                    // If topleft value is much brighter than topright
        lrServo.write(lrServo.read() - 1);                      //  Go to the left by 1 degree
        vDelay(steptime);
      }

      if (downleft - downright < errorValue && downleft - downright > -errorValue) { // If downleft and downright values are about the same, do nothing
        vDelay(steptime);
      }
      else if (downleft > downright) {                          // If downleft value is much darker than downright
        lrServo.write(lrServo.read() + 1);                      // Go to the right by 1 degree
        vDelay(steptime);
      }
      else {                                                    // If downleft value is much brighter than downright
        lrServo.write(lrServo.read() - 1);                      // Go to the left by 1 degree
        vDelay(steptime);
      }

      if (lrServo.read() >= rightThreshold) {                   // if the right threshold is reached, do this maneuver to track the sun
        tdServo.write(60);
        vDelay(1000);
        lrServo.write(leftThreshold + 20);
        vDelay(1000);
      }
      if (lrServo.read() <= leftThreshold) {                    // if the left threshold is reached, do this maneuver to track the sun
        tdServo.write(60);
        vDelay(1000);
        lrServo.write(rightThreshold - 20);
        vDelay(1000);
      }
    }

    else {                                                      // If the panel is 90 degrees and below at the top-down axis (not centeralized)
                                                                // Do the opposite movements to track the sun at the left-right axis
      if (topleft - topright < errorValue && topleft - topright > -errorValue) {
        vDelay(steptime);
      }
      else if (topleft > topright) {
        lrServo.write(lrServo.read() - 1);
        vDelay(steptime);
      }
      else {
        lrServo.write(lrServo.read() + 1);
        vDelay(steptime);
      }

      if (downleft - downright < errorValue && downleft - downright > -errorValue) {
        vDelay(steptime);
      }
      else if (downleft > downright) {
        lrServo.write(lrServo.read() - 1);
        vDelay(steptime);
      }
      else {
        lrServo.write(lrServo.read() + 1);
        vDelay(steptime);
      }

      if (lrServo.read() >= rightThreshold) {
        tdServo.write(120);
        vDelay(1000);
        lrServo.write(leftThreshold + 20);
        vDelay(1000);
      }
      if (lrServo.read() <= leftThreshold) {
        tdServo.write(120);
        vDelay(1000);
        lrServo.write(rightThreshold - 20);
        vDelay(1000);
      }
    }


    if (topleft - downleft < errorValue && topleft - downleft > -errorValue) { // If topleft and downleft values are about the same, do nothing
      vDelay(steptime);
    }
    else if (topleft > downleft) {                              // If topleft value is much darker than downleft
      tdServo.write(tdServo.read() + 1);                        // Move down by 1 degree
      vDelay(steptime);
    }
    else {                                                      // If topleft value is much brighter than downleft
      tdServo.write(tdServo.read() - 1);                        // Move up by 1 degree
      vDelay(steptime);
    }

    if (topright - downright < errorValue && topright - downright > -errorValue) { // If topright and downright values are about the same, do nothing
      vDelay(steptime);
    }
    else if (topright > downright) {                             // If topright value is much darker than downright
      tdServo.write(tdServo.read() + 1);                         // Move down by 1 degree
      vDelay(steptime);
    }
    else {
      tdServo.write(tdServo.read() - 1);                         // If topright value is much brighter than downright
      vDelay(steptime);                                          // Move up by 1 degree
    }

    if (tdServo.read() <= upThreshold) {                         // If the up threshold is reached, stay there
      tdServo.write(upThreshold);
    }
    if (tdServo.read() >= downThreshold) {                       // If the down threshold is reached, stay there
      tdServo.write(downThreshold);
    }

//    Serial.print("topleft:");
//    Serial.println(topleft);
//    Serial.print("topright:");
//    Serial.println(topright);
//    Serial.print("downleft:");
//    Serial.println(downleft);
//    Serial.print("downright:");
//    Serial.println(downright);
//    Serial.print("avgres:");
//    Serial.println(avgres);
//    Serial.print("error:");
//    Serial.println(errorValue);
//    Serial.println();

  }

  Serial.println("Unsuccessful tracking");                      // if solar tracking wasn't successful in the trackingtimeout zone, stay to the last trying position
  trackingState = -1;                                           // Changing the variables that will be sent
  upDownDegrees = tdServo.read();
  leftRightDegrees = lrServo.read();

  digitalWrite(lrServoPower, LOW);                              // Deactivate the servos for power saving
  digitalWrite(tdServoPower, LOW);

  return;
}

//------------- Water level Read -------------------------------------------------------------------------------------------------------

int readWaterLevelSensor() {

  int waterLevelCount = 0;
  unsigned long startTime = 0;
  float counts[waterLevelSample];
  unsigned long sumLevels = 0;
  float avgLevel = 0;


  digitalWrite(waterLevelSensorPower, HIGH);                     // Turn the sensor ON

  vDelay(1000);

  while (waterLevelCount < waterLevelSample) {                   // Taking a measurment every samplingRate seconds
    counts[waterLevelCount] = analogRead(zPin);
    //Serial.println(counts[waterLevelCount]);
    waterLevelCount++;
    vDelay(samplingRate);
  }
  digitalWrite(waterLevelSensorPower, LOW);                      // Turn the sensor OFF

  for (int j = 0; j < waterLevelSample; j++) {                   // Making the sum of the measurments
    sumLevels += counts[j];
  }

  avgLevel = sumLevels / waterLevelSample;                       // Creating the average value



  if (avgLevel < emptyThreshold) {                               // Changing the variables that will be sent
    waterLevelState = -1;
    Serial.print("Water Level: Empty(");

  }
  else if (avgLevel >= emptyThreshold && avgLevel <= waterLowerThreshold) {
    waterLevelState = 0;
    Serial.print("Water Level: Low(");

  }
  else if (avgLevel > waterLowerThreshold && avgLevel <= waterUpperThreshold) {
    waterLevelState = 1;
    Serial.print("Water Level: Medium(");

  }
  else if (avgLevel > waterUpperThreshold) {
    waterLevelState = 2;
    Serial.print("Water Level: High(");

  }
  Serial.print(avgLevel);
  Serial.println(")");

  return avgLevel;                                               // Send current reading
}

//------------- Soil Humidity Read -----------------------------------------------------------------------------------------------------

int readSoilSensor() {

  int soilCount = 0;
  unsigned long startTime = 0;
  float counts[soilSample];
  unsigned long sumSoils = 0;
  float avgSoil = 0;
  int soilPercentage = 0;

  digitalWrite(soilSensorPower, HIGH);                           // Turn the sensor ON

  vDelay(1000);

  while (soilCount < soilSample) {                               // Taking a measurment every samplingRate seconds

    counts[soilCount] = analogRead(zPin);
    //Serial.println(counts[soilCount]);
    soilCount++;
    vDelay(samplingRate);

  }
  digitalWrite(soilSensorPower, LOW);                            // Turn the sensor OFF

  for (int j = 0; j < soilSample; j++) {                         // Making the sum of the measurments
    sumSoils += counts[j];
  }

  avgSoil = sumSoils / (soilSample);                             // Creating the average value

  if((100 - (((avgSoil - lowSoil) / (fullSoil - lowSoil)) * 100)) > 100){
    soilPercentage = 100;
  }
  else if((100 - (((avgSoil - lowSoil) / (fullSoil - lowSoil)) * 100)) < 0){
    soilPercentage = 0;
  }
  else{
    soilPercentage = 100 - (((avgSoil - lowSoil) / (fullSoil - lowSoil)) * 100);     // Creating the percentage
  }
  
  if (soilPercentage <= 2.5) {                                   // Changing the variables that will be sent
    Serial.print("Soil sensor not attached (");
    soilState = -1;
  }
  else if (soilPercentage <= soilThreshold) {
    Serial.print("The soil is DRY (");
    soilState = 0;
  }
  else {
    Serial.print("The soil is WET (");
    soilState = 1;
  }
  Serial.print(soilPercentage);
  Serial.print("%");
  Serial.println(")");


  return soilPercentage;                                         // Sending current reading
}

//------------- Watering  --------------------------------------------------------------------------------------------------------------

void waterPump() {
  if (waterLevel >= emptyThreshold && soilHumidity <= soilThreshold) {
    digitalWrite(waterPumpPower, HIGH);                          // Turn on the water pump if water level isn't empty AND soil is dry

    vDelay(waterPumpDuration * 1000);                            // Waiting waterPumpDuration seconds

    digitalWrite(waterPumpPower, LOW);                           // Turn off the water pump
  }
}
