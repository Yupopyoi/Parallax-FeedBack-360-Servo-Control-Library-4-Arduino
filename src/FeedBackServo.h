#ifndef FEEDBACK360_CONTROL_LIBRARY
#define FEEDBACK360_CONTROL_LIBRARY
#include <Arduino.h>
#include <Servo.h>

class FeedBackServo
{
    public:
        FeedBackServo(const byte feedbackPinNumber = 2);
        void setServoControl(const byte servoPinNumber = 3);
        void setKp(const float _Kp = 1.0);
        void rotate(const int degree, const int threshold = 4);
        int Angle();

    private:
        void pinCheck(const byte pinNumber);
        void static feedback();

        Servo Parallax;
        byte feedbackPinNumber{ 2 };
        volatile int angle;
        float thetaPre;
        unsigned int tHigh, tLow;
        unsigned long rise, fall;
        int turns{ 0 };
        float Kp{ 1.0 };
        const int unitsFC{ 360 };
        const float dcMin{ 0.029 };
        const float dcMax{ 0.971 };
        const int dutyScale{ 1 };
        const int q2min{ unitsFC / 4 };
        const int q3max{ q2min * 3 };
};

#endif
