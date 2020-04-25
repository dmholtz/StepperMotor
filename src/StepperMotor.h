/**
 * Conveniently control stepper motors and calculate acceleration ramps in real-
 * time.
 * 
 * @author dmholtz
 * @version 2.0
 * 
 **/

#ifndef _STEPPER_MOTOR_H_
#define _STEPPER_MOTOR_H_

#include "Arduino.h"

/**
 * Describes the motors rotation behavior. 
 * */
enum class Direction : int8_t
{
	CLOCKWISE = -1,
	COUNTERCLOCKWISE = 1,
	NONE = 0
};

class StepperMotor
{
public:
	/**
	 * Instantiates a new stepper object and sets the steps per revolution
	 * attribute.
	 * 
	 * @param stepsPerRevolution > 0;
	 * */
	StepperMotor(const uint16_t stepsPerRevolution = 200);
	virtual ~StepperMotor() = default;

	/**
	 * Sets the minimum starting speed and computes acceleration ramps.
	 * 
	 * @param minRPM >= 0; Unit: revolution per minute (RPM)
	 * */
	void setMinRPM(const float minRPM);

	/**
	 * Sets the maximum speed. The theoretical maxium speed is limited depending
	 * on the microprocessors clock speed.
	 * 
	 * @param maxRPM > 0; Unit: revolution per minute (RPM)
	 * @requires: maxRPM >= minRPM;
	 * */
	void setMaxRPM(const float maxRPM);

	/**
	 * Sets the acceleration, i.e. the number by which the speed changes every
	 * second. Computes new acceleration ramps.
	 * 
	 * Note: calling this method has no effect if the motor is currently running
	 * 
	 * @requires: !isRunning()
	 * 
	 * @param acceleration > 0; Unit: RPM per second: R*min-1*s-1
	 * */
	void setAcceleration(const float acceleration);

	/**
	 * For convenience: sets all the speed and acceleration parameters at once
	 * by calling each of the corresponding methods.
	 * */
	void speedConfig(const float minRPM, const float maxRPM,
					 const float acceleration);

	/**
	 * Sets the distance, by which the motor should be turned to a new value. 
	 * Does not move the motor itself.
	 * 
	 * @param distance signed distance value 
	 * */
	void setDistance(const long distance);

	/**
	 * @return the (remaining) distance
	 * */
	long getDistance() const;

	/**
	 * Keeps the motor running. Returns true if and only if the motor has
	 * reached its target (i.e. distance == 0)
	 * */
	bool run();

	/**
	 * Returns true if the motor is running and false otherwise.
	 * 
	 * Note: This does not indicate, whether the motor should be running.
	 * @see atTarget();
	 * */
	bool isRunning() const;

	/**
	 * Returns true if and only if the distance is equal to zero. This indicates
	 * that the motor has reached the target.
	 **/
	bool atTarget() const;

	/**
	 * Returns ture if and only if the driver chip is active.
	 * */
	bool isActive() const;

	/**
	 * Activates the driver chip. Implementation depends on chip and must be
	 * implemented in derived class.
	 **/
	virtual void wakeUp() = 0;

	/**
	 * Deactives the driver chip and thus releases the motor. Implementation
	 * depends on chip and must be implemented in derived class.
	 * */
	virtual void release() = 0;

protected:
	/**
	 * Makes the motor physically move by one step into the given direction.
	 * Updates the @member: distance variable:
	 * 	- decrement by one if direction == Direction::CLOCKWISE
	 *  - increment by one if direction == Direction::COUNTERCLOCKWISE
	 *  - don't change variable otherwise
	 * 
	 * The specific implementation depends upon the motor driver and needs to be
	 * implemented by overriding this method.
	 * 
	 * Note: calling step with param Direction::NONE can be used to implement
	 * polling without changing the motors step count
	 * 
	 * @param direction specifies the direction
	 * 
	 * */
	virtual void step(const Direction direction) = 0;

	/// time stamp of the last rising edge
	unsigned long lastRisingEdge{0};

	/// True, if the stepper driver chip is active
	bool active {false};

private:
	const unsigned long TIMER_RESOLUTION = 1000000L;

	/* Hardware ***************************************************************/
	float stepAngle;

	/* Speed configuration ****************************************************/
	float minRPM;
	/// (Minimum) timer count at max speed. Actual timer count must not be less.
	unsigned long maxSpeedCount;
	/// (Maximum) timer count at first step (minimum speed).
	unsigned long zeroCount;

	/* Motor speed / acceleration status **************************************/

	/// Signed number of remaining steps the motor has to move
	long distance{0};
	/// States in which direction the motor is turning
	Direction direction = Direction::NONE;

	/// current count, related to rampStepNumber
	unsigned long currentCount{0};
	/// next count is used to update current count after each step
	unsigned long nextCount{0};
	/// number of step on the acceleration ramp: represented by 'n' in formula
	uint16_t rampStepNumber{0};
	/// indicates, when a new count needs to be calculated
	bool computationDue{true};

	/* Functions to convert units *********************************************/

	/**
	 * Computes and a step angle for a given number of steps per revolution.
	 * 
	 * @param stepsPerRevolution
	 * @return stepAngle in radian
	 * */
	float computeStepAngle(const uint16_t stepsPerRevolution) const;

	/**
	 * Computes a timer count from a given velocity considering this motor's
	 * specification.
	 * 
	 * If the velocity is zero, zero is returned as count (although it  would be
	 * infinity).
	 * If the velocity is below zero, zero is returned (no negative counts).
	 * 
	 * @param velocity >= 0 [radian per second] = [rad*s-1]
	 * @return timer count: always a positive number or zero 
	 * */
	unsigned long countFromVelocity(const float velocity) const;

	/**
	 * Converts a motor's spin rate from revolutions per minute into radian per
	 * second.
	 * 
	 * @param RPM velocity in revolutions per minute
	 * @return radsec: velocity in radian per second 
	 **/
	float radsecFromRPM(const float rpm) const;

	/* Functions for computations *********************************************/

	/**
	 * Returns true if and only if the motor rotates towards the target. In
	 * particular, false is returned if the direction is NONE.
	 * 
	 * @return direction matches 
	 * */
	bool directionTowardsTarget() const;

	/**
	 * To accelerate the motor, the next count is computed using the recursive 
	 * formula. Ensures that the maximum speed is not exceeded.
	 * 
	 * If maxium speed is not exceeded:
	 * 	- increment the rampStepNumber by one
	 *  - compute a new (smaller) value for the next count
	 * Nothing is changed if the maxium speed has been reached.
	 * 
	 * If keepComputing == true: computationDue remains unchanged
	 * If keepComputing == false: computationDue is set to false, indicating 
	 * that no further computation is done before the next step is triggered.
	 * 
	 * @param keepComputing: indicates virtual acceleration
	 * */
	void computeAcceleration(const bool keepComputing = false);

	/**
	 * To decelerate the motor, the next count is computed using the recursive
	 * formuala.
	 * 
	 * If the smallest rampStepNumber is not yet reached:
	 * 	- compute a new (larger) value for the next count
	 *  - decrement the rampStepNumber by one
	 * If the smalles rampStepNumber is reached:
	 * 	- set the current count the the (largest) initialization count
	 *  - set the motor's direction to NONE to indicate the motor has stopped
	 * 
	 * If keepComputing == true: computationDue remains unchanged
	 * If keepComputing == false: computationDue is set to false, indicating 
	 * that no further computation is done before the next step is triggered.
	 * 
	 * @param keepComputing: indicates virtual acceleration
	 * */
	void computeDeceleration(const bool keepComputing = false);

	/**
	 * Reads the current time and triggers a step command if the next step is
	 * due. Updates the distance variable correspondingly.
	 * 
	 * If and only if a step has been triggerd, it sets computationDue to true.
	 * */
	void scheduleStep();

	/**
	 * Returns true if and only if the current step ended. Polls a none-step
	 * command if step has not yet ended.
	 * */
	bool currentStepDone();

};

#endif