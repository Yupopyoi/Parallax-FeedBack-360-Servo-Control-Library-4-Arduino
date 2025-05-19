#include "FeedBackServo.h"
// define feedback signal pin
#define FEEDBACK_PIN1 2
#define FEEDBACK_PIN2 3

// set feedback signal pin number (yellow line)
FeedBackServo servo1 = FeedBackServo(FEEDBACK_PIN1);
FeedBackServo servo2 = FeedBackServo(FEEDBACK_PIN2);

void setup()
{
    Serial.begin(115200);

    servo1.SetServoControl(9);
    servo2.SetServoControl(10);

    servo1.SetTarget(500);
    servo2.SetTarget(300);

    servo1.SetKp(0.1);
    servo2.SetKp(0.1);

    servo1.SetKd(0.05);
    servo2.SetKd(0.05);
}

void loop() 
{  
    servo1.Update();
    servo2.Update();

    Serial.print(servo1.Angle());
    Serial.print(" / ");
    Serial.print(servo2.Angle());
    Serial.print(" / ");
    Serial.print(servo1.Error());
    Serial.print(" / ");
    Serial.println(servo2.Error());
}
