// FeedBack360 Servo Control Library 4 Arduino
// Controll.cpp
// Copyright © Hyoda Kazuaki
// Parallax Feedback 360° High Speed Servo is made by Parallax Inc.
// This library is released under the MIT License.

#include "FeedBackServo.h"

const float FeedBackServo::dcMin = 0.029;
const float FeedBackServo::dcMax = 0.971;
const int FeedBackServo::q2min = FeedBackServo::unitsFC / 4;
const int FeedBackServo::q3max = FeedBackServo::q2min * 3;

FeedBackServo* FeedBackServo::instances[6] = { nullptr };

FeedBackServo::FeedBackServo(byte _feedbackPinNumber)
{
    // feedback pin number validation
    CheckPin(_feedbackPinNumber);
    feedbackPinNumber = _feedbackPinNumber;

    // convert feedback pin number to interrupt number for use on attachInterrupt function
    interruptNumber = digitalPinToInterrupt(feedbackPinNumber);

    if (interruptNumber < 6) {
        instances[interruptNumber] = this;
        switch (interruptNumber)
        {
            case 0: attachInterrupt(0, isr0, CHANGE); break;
            case 1: attachInterrupt(1, isr1, CHANGE); break;
            case 2: attachInterrupt(2, isr2, CHANGE); break;
            case 3: attachInterrupt(3, isr3, CHANGE); break;
            case 4: attachInterrupt(4, isr4, CHANGE); break;
            case 5: attachInterrupt(5, isr5, CHANGE); break;
        }
    }
}

void FeedBackServo::SetServoControl(byte servoPinNumber)
{
    // Servo control pin attach
    Parallax.attach(servoPinNumber);
}

void FeedBackServo::SetKp(float Kp)
{
    FeedBackServo::Kp_ = Kp;
}

void FeedBackServo::SetKd(float Kd)
{
    FeedBackServo::Kd_ = Kd;
}

void FeedBackServo::SetActive(bool isActive)
{
    isActive_ = isActive;
}

void FeedBackServo::SetTarget(int target)
{
    targetAngle_ = target;
}

void FeedBackServo::Update(int threshold = 4)
{
    unsigned long now = micros();
    float dt = (now - lastUpdateMicros_) / 1000000.0f;  // [sec]
    if (dt <= 0.0001f) dt = 0.001f;  // If "dt" is too small, the amount of change in D control is too large
    lastUpdateMicros_ = now;

    int error = targetAngle_ - angle_;
    float errorF = static_cast<float>(error);

    // Re-activate when the error becomes large
    if (!isActive_ && abs(error) > (threshold + 5)) {
        isActive_ = true;
    }

    if (abs(error) <= threshold)
    {
        Parallax.writeMicroseconds(1490);
        previousError_ = 0.0f;
        filteredDerivative_ = 0.0f;
        isActive_ = false;
        return;
    }

    // Proportional Controller
    float P = Kp_ * errorF;

    // Derivative Controller
    float rawDerivative = (errorF - previousError_) / dt;
    filteredDerivative_ = 0.9f * filteredDerivative_ + 0.1f * rawDerivative;
    float D = Kd_ * filteredDerivative_;

    previousError_ = errorF;

    // Output
    float output = constrain(P - D, -200.0f, 200.0f);
    float offset = (error > 0) ? 30.0f : -30.0f;
    float value = output + offset;

    Parallax.writeMicroseconds(1490 - value);
}

int FeedBackServo::Angle()
{
    return angle_;
}

float FeedBackServo::Error()
{
    return previousError_;
}

int FeedBackServo::Turns()
{
    return turns;
}

float FeedBackServo::Theta()
{
    return thetaPre;
}

unsigned int FeedBackServo::THigh()
{
    return tHigh;
}

unsigned int FeedBackServo::TLow()
{
    return tLow;
}

void FeedBackServo::CheckPin(byte _feedbackPinNumber)
{
// Check pin number
#ifdef ARDUINO_AVR_UNO
    if (_feedbackPinNumber != 2 &&
        _feedbackPinNumber != 3)
        exit(1);
#endif
#ifdef ARDUINO_AVR_LEONARDO
    if (_feedbackPinNumber != 0 &&
        _feedbackPinNumber != 1 &&
        _feedbackPinNumber != 2 &&
        _feedbackPinNumber != 3 &&
        _feedbackPinNumber != 7)
        exit(1);
#endif
#ifdef ARDUINO_AVR_MEGA2560
    if (_feedbackPinNumber != 2 &&
        _feedbackPinNumber != 3 &&
        _feedbackPinNumber != 18 &&
        _feedbackPinNumber != 19 &&
        _feedbackPinNumber != 20 &&
        _feedbackPinNumber != 21)
        exit(1);
#endif
}

void FeedBackServo::HandleFeedback()
{
    if (digitalRead(feedbackPinNumber))
    {
        // For more information, please visit https://www.pololu.com/file/0J1395/900-00360-Feedback-360-HS-Servo-v1.2.pdf

        rise = micros();
        if (fall == 0 || rise <= fall) return;
        tLow = rise - fall;

        int tCycle = tHigh + tLow;
        if ((tCycle < 1000) || (tCycle > 1200)) return;

        float dc = (dutyScale * tHigh) / (float)tCycle;

        float theta = ((dc - dcMin) * unitsFC) / (dcMax - dcMin);

        theta = constrain(theta, 0.0f, unitsFC - 1.0f);

        if ((theta < q2min) && (thetaPre > q3max))
            turns++;
        else if ((thetaPre < q2min) && (theta > q3max))
            turns--;

        if (turns >= 0)
            angle_ = (turns * unitsFC) + theta;
        else if (turns < 0)
            angle_ = ((turns + 1) * unitsFC) - (unitsFC - theta);

        thetaPre = theta;
    }
    else
    {
        fall = micros();
        if (rise == 0 || fall <= rise) return;
        tHigh = fall - rise;
    }
}

// ISR delegates
void FeedBackServo::isr0() { if (instances[0]) instances[0]->HandleFeedback(); }
void FeedBackServo::isr1() { if (instances[1]) instances[1]->HandleFeedback(); }
void FeedBackServo::isr2() { if (instances[2]) instances[2]->HandleFeedback(); }
void FeedBackServo::isr3() { if (instances[3]) instances[3]->HandleFeedback(); }
void FeedBackServo::isr4() { if (instances[4]) instances[4]->HandleFeedback(); }
void FeedBackServo::isr5() { if (instances[5]) instances[5]->HandleFeedback(); }
