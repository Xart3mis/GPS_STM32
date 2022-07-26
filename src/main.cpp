#include <Arduino.h>
#include <SoftwareSerial.h>
#include "TinyGPS++.h"

double DistanceThreshold = 500;
double StartLat = 0;
double StartLng = 0;

bool hasFix = false;

TinyGPSPlus gps;
SoftwareSerial ss(PB12, PB13);

void setup()
{
  Serial.begin(9600);
  ss.begin(9600);

  pinMode(PC13, OUTPUT);
  Serial.println("Initializing...");
}

void loop()
{
  Serial.println(gps.satellites.value());

  if (ss.available() > 0)
  {
    char _r = ss.read();

    gps.encode(_r);

    if (gps.location.isUpdated())
    {
      double lat = gps.location.lat();
      double lng = gps.location.lng();

      Serial.print("Latitude= ");
      Serial.print(lat, 6);
      Serial.print(" Longitude= ");
      Serial.println(lng, 6);

      if (Serial.read() == 's')
      {
        StartLat = lat;
        StartLng = lng;
      }

      if (gps.distanceBetween(StartLat, StartLng, lat, lng) > DistanceThreshold)
      {
        digitalWrite(PC13, HIGH);
      }
      else
      {
        digitalWrite(PC13, LOW);
      }
    }
  }
}
