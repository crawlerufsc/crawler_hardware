/* LidarLite v2 laser mounted on 3d printed chassis

  Copyright (c) 2015 Alexander Grau 
  Private-use only! (you need to ask for a commercial-use) 
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  Private-use only! (you need to ask for a commercial-use)
 */

// 
// serial output:
// [0] angle (degree) high byte
// [1] angle (degree) low byte
// [3] distance (cm) high byte
// [4] distance (cm) low byte
// ...repeats
//
// ...until zero pos indication: 
// 0xCC, 0xDD, 0xEE, 0xFF


// Arduino Nano wiring: 
// A5  - Lidar SCL 
// A4  - Lidar SDA
// D4  - Lidar Mode (Add Pull-Down!)  NOT USED!
// D3  - disc encoder pos 
// D5  - disc encoder LED (Active Pull-Down )  NOT USED!


// diagnostic mode: open Arduino IDE serial console (CLTR+SHIFT+M), and send 'd' to the Lidar
// diagnostic output should be like this:
// time=239  freq=3  angleMin=0  angleMax=363
// time=240  freq=3  angleMin=1  angleMax=363
// time=241  freq=3  angleMin=1  angleMax=364


#include <Arduino.h>
#include <Wire.h>
#include "LIDARLite.h"
#include "TimerOne.h"


int pinLidarMode   = 4;
int pinEncoderPos  = 3;
int pinEncoderLED  = 5;
int pinLED  = 13;


#define MODE_NORMAL  0
#define MODE_DEBUG   1


char mode = MODE_NORMAL;
LIDARLite myLidarLite;
volatile int angle = 0;
volatile float ratio = 0;
volatile bool zeroPos = false;
volatile int tickLowDuration = 0;
volatile int tickHighDuration = 0;
volatile int tickAngle = 0;
volatile unsigned long tickLastTime = 0;
int measurements = 0;
unsigned long nextSerialTime = 0;
unsigned long nextInfoTime = 0;
unsigned long rotationCounter = 0;
int angleMax = 0;
int angleMin = 30000;
int distMax = 0;
int distMin = 0;


void timer(){
  angle++; // set to next angle step
}

void encoderTick(){
  volatile byte state = digitalRead(pinEncoderPos);  
  volatile unsigned long ms = millis();          
  //digitalWrite(pinLED, state);
  volatile int tickDuration = ms-tickLastTime;   // compute transition time        
  if (tickDuration == 0) tickDuration = 1;
  tickAngle+=12;    
  if (state == LOW){ 
    // high->low transition
    tickHighDuration = tickDuration;   // remember high time         
    ratio = ((float)tickLowDuration)/((float)tickHighDuration);    
    if (ratio < 0.66){    
      // zero pos      
      zeroPos = true;
      tickAngle = 0;     // reset angle to zero                     
      angle = tickAngle;
    } else {
      angle = tickAngle;
      Timer1.setPeriod(tickDuration*1000/12); // set degree timer based on transition time                         
    }
  } else { 
    // low->high transition
    tickLowDuration = tickDuration;   // remember low time                
  }     
  tickLastTime = ms; // remember transition time  
}

void setup() {
  
  Wire.begin();
    
  pinMode(pinLidarMode, INPUT);  
  pinMode(pinEncoderPos, INPUT);
  //pinMode(pinEncoderLED, OUTPUT);
  pinMode(pinLED, OUTPUT);  

  Serial.begin(115200);

  delay(500);
  
  // begin(int configuration, bool fasti2c, bool showErrorReporting, char LidarLiteI2cAddress)
  myLidarLite.begin(0, true);        

  for (int i=0; i < 5; i++){
     myLidarLite.distance();
     delay(100);
  }
    
  myLidarLite.beginContinuous();            

  Timer1.initialize(1000000); // 1 second
  Timer1.attachInterrupt(timer);  
    
  //Serial.println("Ready");
  attachInterrupt(digitalPinToInterrupt(pinEncoderPos), encoderTick, CHANGE);    
}


void loop() {
  if (millis() >= nextSerialTime){
    nextSerialTime = millis() + 500;
    if (Serial.available()){
      char ch = Serial.read();
      if (ch == 'd') mode = MODE_DEBUG;
      if (ch == 'n') mode = MODE_NORMAL;
    }  
  }
    
  int d = 0;
  //d = myLidarLite.distance();
  while (digitalRead(pinLidarMode) == HIGH);
  d = myLidarLite.distanceContinuous();     
  //int s = myLidarLite.signalStrength();    
  /*if (v == 0){
    // take 1 reading with preamp stabilization and reference pulse (these default to true)
    d = myLidarLite.distance();
  } else {
    // take reading without preamp stabilization and reference pulse (these read about 0.5-0.75ms faster than with)
    d = myLidarLite.distance(false,false);
  } */ 
  int v = angle;
  if (zeroPos){
    // zero point     
    zeroPos = false;    
    if (mode == MODE_NORMAL){
      /*// fill-up to 360 measurements
      while (measurements < 360){      
        Serial.write(0);  
        Serial.write(0);    
        Serial.write(0);  
        Serial.write(0); 
        Serial.flush();         
        measurements++;
      }*/      
      // send zero sync
      Serial.write(0xCC);  
      Serial.write(0xDD);
      Serial.write(0xEE);
      Serial.write(0xFF);
      Serial.flush();    
    } else {
      //Serial.println(rotationCounter);
      //Serial.flush();    
    }    
    rotationCounter++;
  }

  if (mode == MODE_NORMAL){
    Serial.write(v >> 8);  
    Serial.write(v & 0xFF);    
    Serial.write(d >> 8);  
    Serial.write(d & 0xFF);        
    Serial.flush();    
  } 
  else {        
    if (angle > angleMax) angleMax = angle;
    if (angle < angleMin) angleMin = angle;
    if (d > distMax) distMax = d;
    if (d < distMin) distMin = d;
    
    if (millis() >= nextInfoTime){
      nextInfoTime = millis() + 1000;
      Serial.print("time=");
      Serial.print(millis()/1000);
      Serial.print("  freq=");
      Serial.print(rotationCounter);
      Serial.print("  angleMin=");
      Serial.print(angleMin);
      Serial.print("  angleMax=");
      Serial.print(angleMax);
      Serial.print("  distMin=");
      Serial.print(distMin);
      Serial.print("  distMax=");
      Serial.print(distMax);
      Serial.print("  measurements=");
      Serial.print(measurements);
      Serial.println();
      angleMax=0;
      angleMin=30000;
      distMax = 0;
      distMin = 9999;
      rotationCounter=0;
      measurements = 0;      
    }    
  }  
  measurements++;
} 




