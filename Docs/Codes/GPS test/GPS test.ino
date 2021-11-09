#include <TinyGPS++.h>
#include <SoftwareSerial.h>

int rx = 10, tx = 11;

double lt; 
double ln;
double alt;
double spd;


TinyGPSPlus gps;  // The TinyGPS++ object
SoftwareSerial Serial_gps(rx, tx);  // The serial connection to the GPS device

void setup()
{
  Serial.begin(115200);
  Serial_gps.begin(9600);

  /*
  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPS++ with an attached GPS module"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  */
  
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial_gps.available() > 0)
    if (gps.encode(Serial_gps.read()))
       displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    //Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void displayInfo()
{
  if (gps.location.isValid())
  {
    ln = gps.location.lng();
    lt = gps.location.lat();
    Serial.print(ln);
  }
  if (gps.altitude.isValid())
  {
    alt = gps.altitude.meters();
  }
  if (gps.speed.isValid())
  {
    spd = gps.speed.mps();
  }
 
 
  /*
  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  */

  Serial.println();
}
