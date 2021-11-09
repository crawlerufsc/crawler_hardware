#include <AS5600.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <Wire.h>

AS5600 encoder;

long revolutions = 0;   // number of revolutions the encoder has made
double position = 0;    // the calculated value the encoder is at
double lastPosition;
double output;          // raw value from AS5600
long lastOutput;        // last output from AS5600

unsigned long encoder_previous = 0;
unsigned long encoder_now;

float vel;
double pos;

MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float servo_y, servo_p, servo_r;

int j = 0;
float correct;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() 
{
  mpuInterrupt = true;
}


void setup()
{
  
  Wire.begin();
  Wire.setClock(400000); 
  Serial.begin(115200);
  
  mpu.initialize();
  
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
  
  if (devStatus == 0) 
  {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  
}


void loop() 
{
  
  imu_dmp();
  encoder_output();

  
  Serial.print("y: ");
  Serial.print(ypr[0]);
  Serial.print("\t");
  Serial.print("p: ");
  Serial.print(ypr[1]);
  Serial.print("\t");
  Serial.print("r: ");
  Serial.print(ypr[2]);
  Serial.print("\t");
  Serial.print("pos: ");
  Serial.print(pos);
  Serial.print("\t");
  Serial.print("v: ");
  Serial.print(vel);
  Serial.print("\t");
  Serial.println();
  
    
}

void imu_dmp()
{
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 


  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  ypr[0] = ypr[0] * 180 / M_PI;
  ypr[1] = ypr[1] * 180 / M_PI;
  ypr[2] = ypr[2] * 180 / M_PI;

  if (j <= 300) 
    {
      correct = ypr[0]; 
      j++;
    }
    else 
    {
      ypr[0] = ypr[0] - correct; 
    }

  int servo_y = map(ypr[0], -90, 90, 0, 180);
  int servo_p = map(ypr[1], -90, 90, 0, 180);
  int servo_r = map(ypr[2], -90, 90, 180, 0);
  
  /*
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[2] * 180 / M_PI);
  Serial.println();
  */

  /*
  mpu.dmpGetAccel(&aa, fifoBuffer);
  Serial.print("\tRaw Accl XYZ\t");
  Serial.print(aa.x);
  Serial.print("\t");
  Serial.print(aa.y);
  Serial.print("\t");
  Serial.print(aa.z);
  mpu.dmpGetGyro(&gy, fifoBuffer);
  Serial.print("\tRaw Gyro XYZ\t");
  Serial.print(gy.x);
  Serial.print("\t");
  Serial.print(gy.y);
  Serial.print("\t");
  Serial.print(gy.z);
  Serial.println();
  */
 
  }
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

  pos = position*360/4094;

  /*
  Serial.print("Position: ");
  Serial.print((position)*360/4094);
  Serial.print(" Speed: ");
  Serial.print(vel);
  Serial.println(" rpm");
  */

}
