
// Include Guard
#ifndef _A4988_H_
#define _A4988_H_

#include "Arduino.h"
#include "StepperMotor.h"

class A4988 : public StepperMotor
{
public:
    A4988(const uint8_t stepPin, const uint8_t directionPin, const uint8_t standByPin,
          const uint16_t stepsPerRevolution  = 200);
    virtual ~A4988();

    virtual void wakeUp() override;
    virtual void release() override;

protected:
    virtual void step(const Direction direction) override;

private:
    uint8_t stepPin;
    uint8_t directionPin;
    uint8_t standByPin;

    /// delay of the falling edge with respect to lastRisingEdge
    const unsigned long fallingEdgeDelay {10};
};

#endif