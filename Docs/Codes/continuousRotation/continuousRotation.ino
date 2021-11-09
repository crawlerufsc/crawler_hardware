#include <AS5600.h>

AS5600 encoder;

long revolutions = 0;   // number of revolutions the encoder has made
double position = 0;    // the calculated value the encoder is at
double lastPosition;
double output;          // raw value from AS5600
long lastOutput;        // last output from AS5600

unsigned long encoder_previous = 0;
unsigned long encoder_now;

float vel;


void setup()
{
  
  Serial.begin(9600);

}

void loop() 
{
  
  encoder_output();
  
}

void encoder_output()
{

  output = encoder.getPosition();           // get the raw value of the encoder                      
  
  if ((lastOutput - output) > 2047 )        // check if a full rotation has been made
    revolutions++;
  if ((lastOutput - output) < -2047 )
    revolutions--;
    
  position = revolutions * 4096 + output;   // calculate the position the the encoder is at based off of the number of revolutions
  lastOutput = output;   // save the last raw value for the next loop 

  encoder_now = millis();
  if (encoder_now - encoder_previous >= 200) {

    vel = ((position-lastPosition)/(encoder_now-encoder_previous))*14,6555936;
    lastPosition = position;
    encoder_previous = encoder_now;
  }

  Serial.print("Position: ");
  Serial.print((position)*360/4094);
  Serial.print(" Speed: ");
  Serial.print(vel);
  Serial.println(" rpm");

}
