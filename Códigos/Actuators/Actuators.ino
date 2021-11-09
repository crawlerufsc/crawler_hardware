#include <Servo.h>

class _servo
{
  
    Servo servo; // the servo
    int pos; // current servo position
    int init_pos; // servo initial position
    int taget; // servo target position
    int increment; // increment to move for each interval
    int updateInterval; // interval between updates
    unsigned long lastUpdate; // last update of position

  public:

   _servo(int interval)
    {
      updateInterval = interval;
      increment = 1;
    }
    
    void Attach(int pin, int init_pos)
    {
      servo.attach(pin);
      servo.write(init_pos);
    }
    
    void Detach()
    {
      servo.detach();
    }
    
    void Update(int target)
    {
      if ((millis() - lastUpdate) > updateInterval) // time to update
      {
        lastUpdate = millis();
        servo.write(target);
        pos = target;
      }
    }
    
};

_servo servo_front(100);
_servo servo_back(100);
_servo servo_pitch(100);
_servo servo_roll(100);

int init_vel = 255;

int vel;
int pos;
int pitch;
int roll;

#define motor_front 53
#define motor_back 31

void setup() 
{

  Serial.begin(9600);

  servo_front.Attach(51, 45);
  servo_back.Attach(33, 45);
  servo_pitch.Attach(35, 0);
  servo_roll.Attach(49, 0);
  
  pinMode(motor_front, OUTPUT);
  pinMode(motor_back, OUTPUT);

  analogWrite(motor_front, init_vel);
  analogWrite(motor_back, init_vel);

}

void loop() 
{

  servo_front.Update(45-pos);
  servo_back.Update(45-pos);
  servo_pitch.Update(pitch);
  servo_roll.Update(roll);

  analogWrite(motor_front, vel);
  analogWrite(motor_back, vel);
  
}
