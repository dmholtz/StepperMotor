/*
StepperMotor.h: Control a stepper motor with A4988

Created by David M. Holtz
March 2018
*/

#ifndef StepperMotor_h
#define StepperMotor_h

#define TIMER_RESOLUTION 1000000

#define TYPE_A4988 1
#define TYPE_HALFBRIDGE 2

#include "Arduino.h"
#include "DC_Motor.h"

class StepperMotor
{
private:

protected:

public:
	StepperMotor();
	void setMotor(uint8_t step, uint8_t direction, uint8_t ms, uint8_t stby);
	void setHalfBridge(DC_Motor* coil1, DC_Motor* coil2);
	void attachLimitSwitch(uint8_t limitSwitchPin);
	void motorConfig(int spr, long minrpm, long maxrpm, long accS);
	void setVmax(long minrpf, long maxrpm);

	void velocity(int percentage);
	void velocity(int minPercentage, int maxPercentage);
	void setSyncedSpeed(uint8_t percentage);

	bool referencing(long & currentPos_ref);
	void step(int steps);	// blocking
	bool stepping(long & currentPos_ref, long target); // non-blocking real-time acceleration
	void oneStep();
	void oneStep(long * currentPos_ref, int direction);
	void sync(StepperMotor & syncedStepper, long & syncCurrentPos_ref, long syncTarget);
	void endSync();

	void wakeUp();
	void release();

	unsigned long getMaxVelocity();
	unsigned long getMinVelocity();
private:
	// Config parameters
	void paramConfig();
	void stepMotor(int direction);

	// Stepper motor driver type
	uint8_t _driverType;

	// Pin mapping A4988
	uint8_t _stepPin;
	uint8_t _directionPin;
	uint8_t _msPin;
	uint8_t _stbyPin;

	// Half-bridge driver
	DC_Motor *coil1;
	DC_Motor *coil2;

	// Pin mapping limit switch
	uint8_t _limitSwitchPin; 

	// Hardware information
	unsigned long _stepsPerRevolution = 200;	// number of steps per Revoultion
	unsigned long _minStepsPerSecond = 0;		// minimum steps per second (sps) sps = rpm * stepsPerRevolution / 60 min/sec
	unsigned long _maxStepsPerSecond = 0;		// maximum steps per second (sps) sps = rpm * stepsPerRevolution / 60 min/sec
	unsigned long _accelerationDistance = 0;	// distance of acceleration
	int _velocity = 0;							// Percentage of of v-Max
	int _synchedSpeed = 100;					// default percentage
	int _minrpm = 0;
	int _maxrpm = 0;

	// constants
	unsigned long _acceleration = 0;			// acceleration [steps/(s*s)]

	// Timing Counts
	long _zeroCount = 0;						// number of counts between a rising and a falling edge
	long _initialCount = 0;						// seed of recursion
	unsigned long _minimumCount = 0;			// minimum count at maximum speed
	unsigned long _maximumCount = 0;			// maximum count at minimum speed
	unsigned long _currentCount = 0;
	long _lastRisingEdge = 0;

	// stepping
	uint8_t _internalStepNumber = 0;

	bool _firstRun = true;
	bool _stepStatus = false;					// determines the state of the STEP-Pin

	int8_t _direction = 0;
	int8_t _accelerationStatus;					// 0: acceleration, 1: no acceleration, 2: decceleration

	unsigned int _virtualStepNumber = 1;
	int _stepNumber = 0;						// number of the n-th step: increments while acceleration, decrements while decceleration

	// Acceleration tags
	long _distance = 0;							// distance of one stepping operation (including direction)
	unsigned long _stopAccelerationPos = 0;		
	unsigned long _startDeccelerationPos = 0;

	// Stepper synchronization
	bool _syncStatus = false;
	StepperMotor *_syncedStepper;
	long *_syncCurrentPos_ref;
	long _syncTarget;
	int _syncedMotorDirection;
	int _bresenhamError;
	unsigned long _bresenhamDX;	// DX >= DY
	unsigned long _bresenhamDY;
	unsigned long _backupMinsps;
	unsigned long _backupMaxsps;
};

#endif