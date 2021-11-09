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


void setup() {
  
  Serial.begin(115200);
  
  servo_front.Attach(51, 0);
  servo_back.Attach(33, 0);
  servo_pitch.Attach(35, 0);
  servo_roll.Attach(49, 0);
    
}

void loop() {
  
  servo_front.Update(0);
  servo_back.Update(0);
  servo_pitch.Update(0);
  servo_roll.Update(0);
  
}
