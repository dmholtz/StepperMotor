/*
StepperMotor.h: Control a stepper motor with A4988 and half-bridge motor drivers

Created by David M. Holtz
Release notes:

--------------------------------------------------------------------------------
March 2018: 
*new: Acceleration and basic functions

*fixed: various bugfixes
--------------------------------------------------------------------------------
June 2018:
*new: Synchronization of up to two motors

*fixed: Acceleration bugs
--------------------------------------------------------------------------------
March 2019:
*new: added Internal Speed Management 
			method: setVmax(int minrpf, int maxrpm)
			method: velocity(int percentage)
					+overload velocity(int minPercentag, int maxPercentage)
			method: setSynchedSpeed(int percentage)
*new: added option to attach limit switches
*new: added option for auto-referencing

*changed: internal changes in motorConfig()
*/

#include "Arduino.h"
#include "StepperMotor.h"
#include "DC_Motor.h"

StepperMotor::StepperMotor()
{
}

void StepperMotor::setMotor(uint8_t step, uint8_t direction, uint8_t ms, uint8_t stby)
{
	_driverType = TYPE_A4988;

	_stepPin = step;
	pinMode(_stepPin, OUTPUT);
	_directionPin = direction;
	pinMode(_directionPin, OUTPUT);
	_msPin = ms;
	pinMode(_msPin, OUTPUT);
	_stbyPin = stby;
	pinMode(_stbyPin, OUTPUT);
}

void StepperMotor::setHalfBridge(DC_Motor* coil1, DC_Motor* coil2)
{
	_driverType = TYPE_HALFBRIDGE;

	this->coil1 = coil1;
	this->coil2 = coil2;
}

void StepperMotor::attachLimitSwitch(uint8_t limitSwitchPin)
{
	_limitSwitchPin = limitSwitchPin;
	pinMode(_limitSwitchPin, INPUT_PULLUP);
}

void StepperMotor::motorConfig(int spr, long minrpm, long maxrpm, long accS)
{
	StepperMotor::wakeUp(); // default wake up
	_stepsPerRevolution = (long)spr;
	_accelerationDistance = accS;
	StepperMotor::setVmax(minrpm, maxrpm);
}

void StepperMotor::setVmax(long minrpm, long maxrpm)
{
	_minrpm = minrpm;
	_maxrpm = maxrpm;
	velocity(100);
}

void StepperMotor::velocity(int percentage)
{
	if (percentage != _velocity)
	{
		_velocity = percentage;
		_minStepsPerSecond = (long)_minrpm * _stepsPerRevolution / 60;
		_maxStepsPerSecond = (long)_maxrpm * (long)_velocity * _stepsPerRevolution / 6000;
		StepperMotor::paramConfig();
	}
}

void StepperMotor::paramConfig()
{
	// calculate constants
	_acceleration = sq(_maxStepsPerSecond) / 2 / _accelerationDistance;
	float initialCount = TIMER_RESOLUTION * sqrt(2.0 / _acceleration);
	initialCount *= 0.676;
	_initialCount = (long)initialCount;
	_minimumCount = (long)TIMER_RESOLUTION / _maxStepsPerSecond;
	_maximumCount = (long)TIMER_RESOLUTION / _minStepsPerSecond;
	_zeroCount = _minimumCount / 3;

	// virtual acceleration
	_virtualStepNumber = 1;
	while (_initialCount > _maximumCount)
	{
		// simulate acceleration:
		_initialCount -= 2 * _initialCount / (4 * _virtualStepNumber + 1);
		_virtualStepNumber++;
		_accelerationDistance--;
	}
}

void StepperMotor::stepMotor(int direction)
{
	switch (_driverType)
	{
	case TYPE_A4988:
		if (direction > 0)
		{
			digitalWrite(_directionPin, HIGH);
		}
		else
		{
			digitalWrite(_directionPin, LOW);
		}

		if (!_stepStatus)
		{
			_stepStatus = true;
			digitalWrite(_stepPin, HIGH);
		}
		else // stepPin: HIGH
		{
		if (micros() - _lastRisingEdge >= _zeroCount)
		{
			digitalWrite(_stepPin, LOW);
			_stepStatus = false;
		}
		}

		break;

	case TYPE_HALFBRIDGE:
		if (direction > 0 && _internalStepNumber < 3)
		{
			_internalStepNumber++;
		}
		else if (direction > 0 && _internalStepNumber == 3)
		{
			_internalStepNumber = 0;
		}
		else if (direction < 0 && _internalStepNumber > 0)
		{
			_internalStepNumber--;
		}
		else if (direction < 0 && _internalStepNumber == 0)
		{
			_internalStepNumber = 3;
		}

		if (_internalStepNumber == 0)
		{
			coil1->ccw(255);
			coil2->cw(255);
		}
		else if (_internalStepNumber == 1)
		{
			coil1->ccw(255);
			coil2->ccw(255);
		}
		else if (_internalStepNumber == 2)
		{
			coil1->cw(255);
			coil2->ccw(255);
		}
		else if (_internalStepNumber == 3)
		{
			coil1->cw(255);
			coil2->cw(255);
		}
		break;
	}
}

bool StepperMotor::referencing(long &currentPos_ref)
{
	//StepperMotor::wakeUp();
	if (digitalRead(_limitSwitchPin))
	{
		currentPos_ref = 0;
		_firstRun = true;
		return true;
	}
	else
	{
		if(_firstRun)
		{
			_firstRun = false;
			stepMotor(-1);
			currentPos_ref--;
			_lastRisingEdge = micros();
			return false;
		}
		if (micros() - _lastRisingEdge >= _maximumCount)
		{
			stepMotor(-1);
			currentPos_ref--;
			_lastRisingEdge = micros();
			return false;
		}
		return false;
	}
	
}

void StepperMotor::step(int steps)
{
}

bool StepperMotor::stepping(long &currentPos_ref, long target)
{
	StepperMotor::wakeUp();
	if (target == currentPos_ref)
	{
		_firstRun = true;
		return true;
	}
	if (_firstRun) // first run only
	{
		_firstRun = false;
		_stepStatus = false;
		_accelerationStatus = 0;
		long _distance = target - currentPos_ref;

		// check whether another stepper is synced
		if (_syncStatus)
		{
			// initialize Bresenham algorith
			_bresenhamDX = abs(_distance);
			_bresenhamDY = abs(_syncTarget - *_syncCurrentPos_ref);
			_bresenhamError = _bresenhamDX / 2;

			/* If stepper X is significantly faster than synced stepper Y,
			// velocity of stepper X has to be reduced
			_backupMinsps = _minStepsPerSecond;
			_backupMaxsps = _maxStepsPerSecond;
			if (_maxStepsPerSecond > _syncedStepper->getMaxVelocity())
			{
				// reduce velocity of stepper X
				_minStepsPerSecond = _syncedStepper->getMinVelocity();
				_maxStepsPerSecond = _syncedStepper->getMaxVelocity();
				StepperMotor::paramConfig();
			}*/

			// syncedMotorDirection
			if (_syncTarget - *_syncCurrentPos_ref >= 0)
			{
				_syncedMotorDirection = 1;
			}
			else
			{
				_syncedMotorDirection = -1;
			}
		}

		_stepNumber = _virtualStepNumber;
		// determine direction
		if (target > currentPos_ref)
		{
			_direction = 1;
		}
		else
		{
			_direction = -1;
		}
		// first step
		_currentCount = _initialCount;
		stepMotor(_direction);
		_lastRisingEdge = micros();

		if (abs(_distance) <= 2 * _accelerationDistance)
		{
			_stopAccelerationPos = currentPos_ref + _distance / 2;
			_startDeccelerationPos = _stopAccelerationPos;
		}
		else
		{
			if (_distance > 0)
			{
				_stopAccelerationPos = currentPos_ref + _accelerationDistance;
				_startDeccelerationPos = target - _accelerationDistance;
			}
			else
			{
				_stopAccelerationPos = currentPos_ref - _accelerationDistance;
				_startDeccelerationPos = target + _accelerationDistance;
			}
		}

		return false;
	}
	if (_stepStatus == true) // stepPin: HIGH
	{
		if (micros() - _lastRisingEdge >= _zeroCount)
		{
			digitalWrite(_stepPin, LOW);
			_stepStatus = false;
		}
		return false;
	}
	if (_stepStatus == false)
	{
		if (micros() - _lastRisingEdge >= _currentCount)
		{
			currentPos_ref += _direction;
			if (target == currentPos_ref)
			{
				return false;
			}
			else
			{
				stepMotor(_direction);
				_lastRisingEdge = micros();

				if (currentPos_ref == _stopAccelerationPos)
				{
					_accelerationStatus = 1;
				}
				if (currentPos_ref == _startDeccelerationPos)
				{
					_accelerationStatus = 2;
				}

				switch (_accelerationStatus)
				{
				case 0:
					_currentCount -= (2 * _currentCount / (4 * _stepNumber + 1));
					_stepNumber++;
					break;
				case 1:
					break;
				case 2:
					_currentCount = (4 * _stepNumber + 1) * _currentCount / (4 * _stepNumber - 1);
					_stepNumber--;
					break;
				}

				if (_currentCount > _maximumCount)
				{
					_currentCount = _maximumCount;
				}
				else if (_currentCount < _minimumCount)
				{
					_currentCount = _minimumCount;
				}

				if (_syncStatus)
				{
					_bresenhamError -= _bresenhamDY;
					if (_bresenhamError < 0)
					{
						_syncedStepper->oneStep(_syncCurrentPos_ref, _syncedMotorDirection);
						_bresenhamError += _bresenhamDX;
					}
				}
			}
		}
		else
		{
			if (_syncStatus)
			{
				_syncedStepper->oneStep();
			}
		}
	}
	return false;
}

void StepperMotor::oneStep()
{
	if (_stepStatus)
	{
		if (micros() - _lastRisingEdge > 10)
		{
			digitalWrite(_stepPin, LOW);
			_stepStatus = false;
		}
		return;
	}
}

void StepperMotor::oneStep(long *currentPos_ref, int direction)
{
	StepperMotor::wakeUp();
	stepMotor(direction);
	_lastRisingEdge = micros();
	*currentPos_ref += direction;
}

void StepperMotor::sync(StepperMotor &syncedStepper, long &syncCurrentPos_ref, long syncTarget)
{
	_syncedStepper = &syncedStepper;
	_syncCurrentPos_ref = &syncCurrentPos_ref;
	_syncTarget = syncTarget;
	_syncStatus = true;
}

void StepperMotor::endSync()
{
	_syncStatus = false;
}

void StepperMotor::release()
{
	switch (_driverType)
	{
	case TYPE_A4988:
		digitalWrite(_stbyPin, LOW);
		break;
	case TYPE_HALFBRIDGE:
		coil1->stop();
		coil2->stop();
		break;
	}
}

void StepperMotor::wakeUp()
{
	switch (_driverType)
	{
	case TYPE_A4988:
		digitalWrite(_stbyPin, HIGH);
		break;
	case TYPE_HALFBRIDGE:
		stepMotor(0);
		break;
	}
}

unsigned long StepperMotor::getMinVelocity()
{
	return _minStepsPerSecond;
}

unsigned long StepperMotor::getMaxVelocity()
{
	return _maxStepsPerSecond;
}