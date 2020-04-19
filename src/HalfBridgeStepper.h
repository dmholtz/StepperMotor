
// Include Guard
#ifndef _HALFBRIDGESTEPPER_H_
#define _HALFBRIDGESTEPPER_H_

#include "Arduino.h"
#include "DC_Motor.h"
#include "StepperMotor.h"

class HalfBridgeStepper : public StepperMotor
{
private:
	DC_Motor *coil1;
	DC_Motor *coil2;

	uint8_t hardwareStepNumber {0};

protected:
	virtual void step(const Direction direction) override;

public:
	HalfBridgeStepper(const DC_Motor *coil1, const DC_Motor *coil2,
					  const uint16_t stepsPerRevolution = 200);
	virtual ~HalfBridgeStepper();

	virtual void wakeUp() override;
	virtual void release() override;
};

#endif