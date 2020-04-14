
// Include Guard
#ifndef _HALFBRIDGESTEPPER_H_
#define _HALFBRIDGESTEPPER_H_

#include "Arduino.h"
#include "DC_Motor.h"

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

	void wakeUp();
	void release();
};

#endif