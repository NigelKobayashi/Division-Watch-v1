
/*
Division Watch - GPS +  LED functionality
Author: Nigel Kobayashi
1-16-2024

- Updated with ringRun, ring 
TODO
 - IMU tapping to wake up
 - IMU flicking to wake up
 - debug long GPS fix time, display time to watch/serial monitor

*/

#include "LSM6DS3.h"
#include "Wire.h"



#include <Adafruit_GPS.h>
#include <Adafruit_NeoPixel.h>

#define D2 2

const int numPixels = 12;
const int pixelPin = D2;

Adafruit_NeoPixel ring = Adafruit_NeoPixel(numPixels, pixelPin);
uint32_t foreground = ring.Color(255, 0, 0); // r, g, b - blue
uint32_t background = ring.Color(255, 170, 0); // 

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

//Create a instance of class LSM6DS3
LSM6DS3 IMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

uint32_t timer = millis();

void setup() {
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  ring.begin(); // start the NeoPixel display

  //Call .begin() to configure the IMUs
    if (IMU.begin() != 0) {
        Serial.println("IMU error");
    } else {
        Serial.println("IMU OK!");
    }

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
  for (int i = 0; i < numPixels; i++) {
    ring.setPixelColor(i, background);
    ring.setBrightness(20);
    ring.show();  
  }
  //ringGPSLoading();
}

void loop() {
  //readGPSAll();
  //delay(2000);
  //ringFade();
  //ringTimer(10*1000);
  /*

  
  for(int i = 0; i < 360; i+=15){
    ringBearing(i);
    Serial.println(i);
    delay(2000);
  }
  */
  /*
  float angleX = IMU.readFloatAccelY();
  float angleY = IMU.readFloatAccelY();
  float angleZ = IMU.readFloatAccelZ();

  Serial.print(angleX);
  Serial.print(", ");
  Serial.print(angleY);
  Serial.print(", ");
  Serial.println(angleZ);
  */


  LEDLevel();
  
  
  /*
  Serial.println("Got a GPS fix.");
  readGPSTime();
  ringRun();
  delay(5000);
  */
}

void ringGPSLoading(){
  /*
  for (int i = 0; i < numPixels; i++) {
    ring.setPixelColor(i, background);
    ring.setBrightness(20);
    ring.show();  
  }*/
  while(!GPS.fix){
    Serial.println("GPS searching for fix.");
    readGPSAll();
    ringFade();
  } 
  Serial.println("found");
}

void ringFade(){
  uint8_t min = 1;
  uint8_t max = 20;
  float fadeTime = 3.0;
  float loopTime = (fadeTime*1000/2) / (max-min); //how many ms to fit in the fadeTime.
  //Serial.println(loopTime);
  for (int i = 0; i < numPixels; i++) {
    ring.setBrightness(min);
    ring.setPixelColor(i, background);
    ring.show();                     
  }

  for (int j = min; j < max; j++) {
    ring.setBrightness(j);
    ring.show();
    delay(loopTime);
  }
  for (int k = max; k > min; k--) {
    ring.setBrightness(k);
    ring.show();
    delay(loopTime);
  } 
}

void ringRun(){
  // blue dot circles around a white background (for PixelRing 24)
  for (int i = 0; i < numPixels; i++) {
    ring.setBrightness(20);
    ring.setPixelColor(i, foreground); // set pixel i to foreground
    ring.show();                       // actually display it
    delay(50);                         // milliseconds 
    ring.setPixelColor(i, background); // set pixel to background before moving on
    ring.show(); 
  }
}

void ringTimer(float length){
  float startTime = millis();
  for (int i = 0; i < numPixels; i++) {
    ring.setBrightness(20);
    ring.setPixelColor(i, background);
    ring.show();                     
  }
  while((millis() - startTime) < length){
    float targetLED = ((millis()-startTime) / length) * numPixels;
    Serial.print((millis() - startTime));
    Serial.print(", ");
    Serial.println((int)targetLED);
    if(targetLED > numPixels) targetLED = 0;

    for (int i = 0; i < numPixels; i++) {
      ring.setBrightness(20);
      if(i == (int) targetLED){ 
        ring.setPixelColor(i, foreground);
      }else ring.setPixelColor(i, background);
      ring.show();                     
    }
  }
}

void ringBearing(int angle){
  int targetLED = ((float)angle /360) * (float)numPixels;
  if(targetLED < 0) targetLED = (numPixels-1) + targetLED;
  if(targetLED > numPixels) targetLED = 0;

  for (int i = 0; i < numPixels; i++) {
    ring.setBrightness(20);
    if(i == targetLED){ 
      ring.setPixelColor(i, foreground);
    }else ring.setPixelColor(i, background);
    ring.show();                     
  }
}
//convert accleration (g) to an angle using two axes of accelerometer
//accelVal1, accelVal2 = two accelearometer axes readings (g)
//returns float angle of the angle between about the third axis
float accelToAngle(float accelVal1, float accelVal2){
  Serial.print(accelVal1);
  Serial.print(", ");
  Serial.print(accelVal2);
  Serial.print(" , ");
  float angle = -1;
  angle = atan2(accelVal1, accelVal2);
  angle = (angle * 180) / 3.14159265; //convert rads to deg
  Serial.println(angle);
  return angle;
}

void readIMUAll(){
  //Accelerometer
    //Serial.print("\nAccelerometer:\n");
    //Serial.print(" X1 = ");
    //Serial.print(IMU.readFloatAccelX(), 4);
    //Serial.print(", ");
    //Serial.print(IMU.readFloatAccelY(), 4);
    //Serial.print(" , ");
    //Serial.println(IMU.readFloatAccelZ(), 4);
/*
    //Gyroscope
    Serial.print("\nGyroscope:\n");
    Serial.print(" X1 = ");
    Serial.println(IMU.readFloatGyroX(), 4);
    Serial.print(" Y1 = ");
    Serial.println(IMU.readFloatGyroY(), 4);
    Serial.print(" Z1 = ");
    Serial.println(IMU.readFloatGyroZ(), 4);

    //Thermometer
    Serial.print("\nThermometer:\n");
    Serial.print(" Degrees C1 = ");
    Serial.println(IMU.readTempC(), 4);
    Serial.print(" Degrees F1 = ");
    Serial.println(IMU.readTempF(), 4);
    */
}
void readGPSTime(){
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  if (GPS.fix) {
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
  }

}

void LEDLevel(){  
  float angleY = IMU.readFloatAccelY();
  float angleZ = IMU.readFloatAccelZ();
  ringBearing((int) accelToAngle(angleY, angleZ));
  delay(100);
}

void readGPSAll(){
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  //if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
    }
  //}
}
