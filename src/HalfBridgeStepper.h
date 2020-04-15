
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

	uint8_t hardwareStepNumber;
protected:
	virtual void hardwareStep(const int8_t direction) override;

public:
	HalfBridgeStepper(const DC_Motor* coil1, const DC_Motor* coil2);
	virtual ~HalfBridgeStepper();

	virtual void wakeUp() override;
	virtual void release() override;
};

#endif