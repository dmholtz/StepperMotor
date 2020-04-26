#include "StepperMotor.h"

// #define STEPPER_MOTOR_ENABLE_DEBUG_PRINT

StepperMotor::StepperMotor(const uint16_t stepsPerRevolution)
{
	if (stepsPerRevolution == 0)
	{
#ifdef STEPPER_MOTOR_ENABLE_DEBUG_PRINT
		Serial.println("Illegal constructor call: steps per revolution = 0");
#endif
		stepAngle = computeStepAngle(1); // Avoid by-zero division
	}
	else
	{
		stepAngle = computeStepAngle(stepsPerRevolution);
	}
	// Initialize to default values to prevent the program from running into an
	// illegal state
	speedConfig(1.0, 2.0, 1.0);
}

void StepperMotor::setMinRPM(const float minRPM)
{
	if (minRPM > 0)
	{
		this->minRPM = minRPM;
		float minRadsec = radsecFromRPM(minRPM);
		minSpeedCount = countFromVelocity(minRadsec);
	}
}

void StepperMotor::setMaxRPM(const float maxRPM)
{
	if (maxRPM > 0)
	{
		float maxRadsec = radsecFromRPM(maxRPM);
		maxSpeedCount = countFromVelocity(maxRadsec);
	}
}

void StepperMotor::setAcceleration(const float acceleration)
{
	if (!this->isRunning() && acceleration > 0)
	{
		// see also equation 15
		float accRadsec2 = radsecFromRPM(acceleration);
		zeroCount = 0.676 * TIMER_RESOLUTION *
					sqrt(2 * stepAngle / accRadsec2);
	}
}

void StepperMotor::speedConfig(const float minRPM, const float maxRPM,
							   const float acceleration)
{
	setMinRPM(minRPM);
	setMaxRPM(maxRPM);
	setAcceleration(acceleration);
}

void StepperMotor::setDistance(const long distance)
{
	this->distance = distance;
}

long StepperMotor::getDistance() const
{
	return distance;
}

bool StepperMotor::run()
{
	if (this->atTarget() && !this->isRunning())
	{
		computationDue = true;
		return currentStepDone();
	}
	else
	{
		if (computationDue)
		{
			this->wakeUp();
			if (!this->isRunning())
			{
				// Initial acceleration has to be called before setting a direction
				computeAcceleration();
				if (distance > 0)
				{
					direction = Direction::COUNTERCLOCKWISE;
				}
				else
				{
					direction = Direction::CLOCKWISE;
				}
			}
			else if (directionTowardsTarget())
			{
				if (abs(distance) > rampStepNumber)
				{
					computeAcceleration();
				}
				else
				{
					computeDeceleration();
				}
			}
			else
			{
				computeDeceleration();
			}
		}
		scheduleStep();
		return false;
	}
}

bool StepperMotor::isRunning() const
{
	return direction != Direction::NONE;
}

bool StepperMotor::atTarget() const
{
	return distance == 0;
}

long StepperMotor::runConstSpeed(const Direction direction)
{
	this->direction = direction;
	if (direction == Direction::NONE)
	{
		currentStepDone();
		constSpeedDistance = 0;
	}
	else
	{
		// rotation either clockwise or counterclockwise
		currentCount = minSpeedCount;
		unsigned long currentTime = micros();
		if ((currentTime - lastRisingEdge) >= currentCount)
		{
			constSpeedDistance += static_cast<int8_t>(direction);
			this->step(direction);
			lastRisingEdge = currentTime;
		}
		else
		{
			this->step(Direction::NONE);
		}
	}
	return constSpeedDistance;
}

long StepperMotor::getConstSpeedDistance() const
{
	return constSpeedDistance;
}

bool StepperMotor::isActive() const
{
	return active;
}

float StepperMotor::computeStepAngle(const uint16_t stepsPerRevolution) const
{
	return 2.0 * PI / stepsPerRevolution;
}

unsigned long StepperMotor::countFromVelocity(const float velocity) const
{
	if (velocity <= 0.0)
	{
		return 0L;
	}
	else
	{
		return (stepAngle / velocity) * TIMER_RESOLUTION;
	}
}

float StepperMotor::radsecFromRPM(const float rpm) const
{
	return rpm * PI / 30.0; // simplified: rpm * 2 * PI / 60
}

bool StepperMotor::directionTowardsTarget() const
{
	if (direction == Direction::COUNTERCLOCKWISE && distance >= 0)
	{
		return true;
	}
	else if (direction == Direction::CLOCKWISE && distance <= 0)
	{
		return true;
	}
	return false;
}

void StepperMotor::computeAcceleration(const bool keepComputing)
{
	if (!this->isRunning())
	{
		currentCount = 0;
		nextCount = zeroCount;
		rampStepNumber = 0;
	}
	else
	{
		rampStepNumber++;
		nextCount = (uint32_t)(currentCount - 2.0 * currentCount / (4.0 * rampStepNumber + 1.0));
		if (nextCount < maxSpeedCount)
		{
			// Exceeding maximum speed, undo acceleration
			rampStepNumber--;
			nextCount = currentCount;
		}
	}
	computationDue = keepComputing;
}

void StepperMotor::computeDeceleration(const bool keepComputing)
{
	if (rampStepNumber > 0)
	{
		nextCount = (uint32_t)(currentCount - 2.0 * currentCount / (-4.0 * rampStepNumber + 1));
		rampStepNumber--;
	}
	else
	{
		direction = Direction::NONE;
		nextCount = zeroCount;
	}
	computationDue = keepComputing;
}

void StepperMotor::scheduleStep()
{
	unsigned long currentTime = micros();
	if ((currentTime - lastRisingEdge) >= currentCount)
	{
		distance -= static_cast<int8_t>(direction);
		this->step(direction);
		lastRisingEdge = currentTime;
		currentCount = nextCount;
		computationDue = true;
	}
	else
	{
		this->step(Direction::NONE);
	}
}

bool StepperMotor::currentStepDone()
{
	unsigned long currentTime = micros();
	if ((currentTime - lastRisingEdge) >= currentCount)
	{
		return true;
	}
	else
	{
		// Polls a none-step command (required by some drivers)
		this->step(Direction::NONE);
		return false;
	}
}