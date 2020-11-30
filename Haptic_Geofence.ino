/*
 * This is Haptic Geofence library – developed for haptic communication based on the GPS data and a predefined static geofence. 
 * Originally tested with 
 * Adafruit GPS Module (Product ID: 746)
 * Adafruit Haptic Motor Controller (Product ID: 2305)
 * Adafruit Flora Microcontroller (Product ID: 659)
 * 
 * The current code includes geofence calculations based on a desired radius from a fixed location coordinates using the Haversine formula. 
 * It also triggers a haptic feedback when the GPS position is determined to be inside the geofence.
 * 
 * Modified by Neeraj Yadav for M.S. Thesis (Copyright 2017) http://hdl.handle.net/1969.1/187304
 * 
 * Original Code by Limor Fried/Ladyada for Adafruit Industries can be found in Adafruit GPS Library https://github.com/adafruit/Adafruit_GPS
 * 
 * Check license.txt for more information. 
 * All previous credits must be mentioned in any redistribution. 
 * This open source code is available primarily for educational purposes.
 */

#include <Adafruit_GPS.h>
#include <math.h>
#include <Wire.h>
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;


double PointLat = 51.1789;   // Latitude of Point A
double PointLon = 78.0421;   // Longitude of Point A
double R = 6365.229;           // Radius of Earth at Latitude 51.1789 and Altitude 100m above sea level.

double lat1, lon1;
double latR1, latR2, lonR1, lonR2, dlon, dlat;
double a, e, d;

#define GPSSerial Serial1  // Name of the hardware serial port
Adafruit_GPS GPS(&GPSSerial); // Connect to the GPS on the hardware port
#define GPSECHO false  // 'true' if required to debug and listen to the raw GPS sentences

uint32_t timer = millis();
uint8_t effect = 1;

void setup()
{
      
  Serial.begin(115200); // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  GPS.begin(9600);  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status
  
  drv.begin(); // Haptic Library
  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_INTTRIG); 
  
  delay(1000);
  
  GPSSerial.println(PMTK_Q_RELEASE); // Ask for firmware version
  drv.begin();
  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_INTTRIG);  // default, internal trigger when sending GO command
 
}

void loop() // Runs over and over again
{  
  char c = GPS.read(); // read data from the GPS in the 'main loop'
  if (GPSECHO)
    if (c) Serial.print(c);
    
  if (GPS.newNMEAreceived()) 
  {
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // If fail to parse a sentence, wait for another
  
  }
  
  if (timer > millis()) timer = millis(); // Resetting if millis() or timer wraps around
  
  if (millis() - timer > 100) // Run every 100  milliseconds
  {  
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    
    if (GPS.fix) 
    {
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 6); Serial.print(GPS.lat);
        Serial.print(", "); 
        Serial.print(GPS.longitude, 6); Serial.println(GPS.lon);
        Serial.print("Location (in degrees, works with Google Maps): ");
        Serial.print(GPS.latitudeDegrees, 6);
        Serial.print(", "); 
        Serial.println(GPS.longitudeDegrees, 6);
        
        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

          if( calcDist() <= 20.0) // Start the Haptic function if inside the Geofence
          {
                  Serial.println("You are inside the Geofence. Kindly GET OUT asap!");
                  drv.setWaveform(0, effect);  // play effect 
                  drv.setWaveform(1, 0);       // end waveform
                  drv.go();
                  
                  delay(10);

                   effect++;
                   if (effect > 3) effect = 1;              
           }      
     }
   }
}




// **Geofence Calculations**

double convertDegMinToDecDeg (float degMin)  // Converting lat/long from degree-minute format to decimal-degrees
{     
  double min = 0.0;
  double decDeg = 0.0;
 
  min = fmod((double)degMin, 100.0);
 
  degMin = (int) ( degMin / 100 );  //Rebuilding coordinates in decimal degrees
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}

double calcDist()  // Haversine based distance calculation formula - Circle
{    
  lon1 = convertDegMinToDecDeg(GPS.longitude);
  lat1 = convertDegMinToDecDeg(GPS.latitude);
  
  lonR1 = lon1*(PI/180);    // Converting the current GPS coords from decDegrees to Radians
  lonR2 = PointLon*(PI/180);
  latR1 = lat1*(PI/180);
  latR2 = PointLat*(PI/180);

  dlon = lonR2 - lonR1;
  dlat = latR2 - latR1;

  a = (sq(sin(dlat/2))) + cos(latR1) * cos(latR2) * (sq(sin(dlon/2)));   // Haversine Formula. Result in meters.
  e = 2 * atan2(sqrt(a), sqrt(1-a)) ;  
  d = R * e * 1000;

  Serial.println();
  Serial.print("Distance to the Point(M): ");
  Serial.println(d, 6);
  Serial.println();
  return d;

}
