#include "HalfBridgeStepper.h"

HalfBridgeStepper::HalfBridgeStepper(const DC_Motor *coil1, const DC_Motor *coil2)
	: coil1{coil1}, coil2{coil2}, hardwareStepNumber{0}
{
}

HalfBridgeStepper::~HalfBridgeStepper()
{
	this->release();
	delete coil1;
	delete coil2;
}

/**
 * Makes the motor step by one step into the desired direction. If direction is
 * zero, the coils are active but the rotor does not turn.
 * 
 * @param direction 
 * 
 * */
void HalfBridgeStepper::hardwareStep(const int8_t direction)
{
	if (direction > 0)
	{
		++hardwareStepNumber;
	}
	else if (direction < 0)
	{
		--hardwareStepNumber;
	}
	
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
	coil1->stop();
	coil2->stop();
}

void HalfBridgeStepper::wakeUp()
{
	this->hardwareStep(0);	// does not perform an acutal step but activates the coils only
}