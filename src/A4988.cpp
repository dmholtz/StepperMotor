#include "A4988.h"

A4988::A4988(const uint8_t stepPin, const uint8_t directionPin, const uint8_t standByPin,
             const uint16_t stepsPerRevolution)
    : StepperMotor(stepsPerRevolution), stepPin{stepPin}, directionPin{directionPin}, standByPin{standByPin}
{
    pinMode(stepPin, OUTPUT);
    pinMode(directionPin, OUTPUT);
    pinMode(standByPin, OUTPUT);
}

A4988::~A4988()
{
    this->release();
    digitalWrite(directionPin, LOW);
    digitalWrite(stepPin, LOW);
}

void A4988::step(const Direction direction)
{
    if (direction == Direction::NONE)
    {
        if (micros() > (lastRisingEdge + fallingEdgeDelay))
        {
            digitalWrite(stepPin, LOW);
        }
        return;
    }
    else if (direction == Direction::CLOCKWISE)
    {
        digitalWrite(directionPin, LOW);
    }
    else
    {
        digitalWrite(directionPin, HIGH);
    }
    digitalWrite(stepPin, HIGH);    
}

void A4988::release()
{
    active = false;
    digitalWrite(standByPin, LOW);
}

void A4988::wakeUp()
{
    active = true;
    digitalWrite(standByPin, HIGH);
}