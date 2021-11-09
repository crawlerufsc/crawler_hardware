#include <Wire.h>
#include <AS5600.h>
#include <LIDARLite.h>

#define pwm  5    // Arduino PWM pin that is connected to the base of the transistor
int pwm_vel;      //Variable that will store the current PWM value (0 -> 255)

LIDARLite myLidarLite;
#define mode 3    // Arduino pin that is connected to the MODE pin of the Lidar

AS5600 encoder;

long revolutions = 0;   // Number of revolutions the encoder has made
double position = 0;    // The calculated value the encoder is at
double lastPosition;
double output;          // Raw value from AS5600
long lastOutput;        // Last output from AS5600

unsigned long encoder_previous = 0;
unsigned long encoder_now;

float vel;

void setup() {
  
  Serial.begin(115200);
  
  myLidarLite.begin();
  myLidarLite.beginContinuous();

  //  Configures the state of the previously declared pins
  pinMode(pwm, OUTPUT);
  pinMode(mode, INPUT);
  
  analogWrite(pwm, 255);
  
}

void loop() {
  
  output = encoder.getPosition();           // Get the raw value of the encoder                      
  
  if ((lastOutput - output) > 2047 )        // Check if a full rotation has been made
    revolutions++;
  if ((lastOutput - output) < -2047 )
    revolutions--;
    
  position = revolutions * 4096 + output;   // Calculate the position the the encoder is at based off of the number of revolutions
  lastOutput = output;                      // Save the last raw value for the next loop 

  encoder_now = millis();
  if (encoder_now - encoder_previous >= 200) {

    vel = ((position-lastPosition)/(encoder_now-encoder_previous))*14,6555936;
    lastPosition = position;
    encoder_previous = encoder_now;
  }

  if(!digitalRead(2)){
    Serial.print("Distance: ");
    Serial.print(myLidarLite.distanceContinuous());
    Serial.print(" Position: ");
    Serial.print(int((output)*360/4094));
    Serial.print(" Speed: ");
    Serial.print(vel);
    Serial.println(" rpm");
  }
}
