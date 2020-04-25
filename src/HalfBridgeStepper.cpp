#include "HalfBridgeStepper.h"

HalfBridgeStepper::HalfBridgeStepper(const DC_Motor *coil1, const DC_Motor *coil2,
									 const uint16_t stepsPerRevolution)
	: StepperMotor(stepsPerRevolution), coil1{coil1}, coil2{coil2}
{
}

HalfBridgeStepper::~HalfBridgeStepper()
{
	this->release();
}

/**
 * Makes the motor step by one step into the desired direction. If direction is
 * zero, the coils are active but the rotor does not turn.
 * 
 * @param direction 
 * 
 * */
void HalfBridgeStepper::step(const Direction direction)
{
	// Increments or decrements hardwareStepNumber, depending on direction
	hardwareStepNumber += static_cast<int8_t>(direction);

	// Select coil powering from hardwareStepNumber table
	if ((hardwareStepNumber & B11) == B00)
	{
		coil1->ccw(255);
		coil2->cw(255);
	}
	else if ((hardwareStepNumber & B11) == B01)
	{
		coil1->ccw(255);
		coil2->ccw(255);
	}
	else if ((hardwareStepNumber & B11) == B10)
	{
		coil1->cw(255);
		coil2->ccw(255);
	}
	else // if ((hardwareStepNumber & B11) == B11)
	{
		coil1->cw(255);
		coil2->cw(255);
	}
}

void HalfBridgeStepper::release()
{
	active = false;
	coil1->stop();
	coil2->stop();
}

void HalfBridgeStepper::wakeUp()
{
	active = true;
	// does not perform an acutal step but activates the coils only
	this->step(Direction::NONE);
}