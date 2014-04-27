/* 
* SkidSteering.h
*
* Created: 2014-04-26 22:22:56
* Author: pjp
*/
#include "Arduino.h"

#ifndef __SKIDSTEERING_H__
#define __SKIDSTEERING_H__

#define	FULL_RANGE_INPUT	255
#define HALF_RANGE_INPUT	(FULL_RANGE_INPUT / 2)
#define CENTRE_INPUT		HALF_RANGE_INPUT
#define DEAD_ZONE			(FULL_RANGE_INPUT / 50)

#define FULL_PULSE_WIDTH			(MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
#define HALF_PULSE_WIDTH			(MIN_PULSE_WIDTH + (FULL_PULSE_WIDTH / 2))
#define QUARTER_PULSE_WIDTH			(MIN_PULSE_WIDTH + (FULL_PULSE_WIDTH / 4))
#define THREE_QUARTERS_PULSE_WIDTH	(MIN_PULSE_WIDTH + (3 * (FULL_PULSE_WIDTH / 4)))

#define	MIN_PULSE_WIDTH		1000
#define	MAX_PULSE_WIDTH		2000
#define	WAIT_FOR_PULSE		20000

#define TICK_DELAY_MS		250		// How long (in mS) to delay in each loop
#define DIRECTION_CHANGE_DELAY_MS	500		// How long (in mS) to delay after a direction change

#define VOLTS_PER_AMP		1.65

const uint8_t REFERENCE_VOLTS	=	5;   // the default reference on a 5-volt board
const float VOLTS_PER_BIT		=	REFERENCE_VOLTS / 1024.0;
const float RESISTOR_FACTOR		=	1024.0 / 2;

const int MIN_MILLI_VOLTS			=	6600;	// 6 * 1.1
const int MAX_MILLI_VOLTS			=	8400;	// 6 * 1.4

struct MotorPinDefinition {
	uint8_t motorAmps;
	uint8_t motorBrake;
	uint8_t motorDirection;
	uint8_t motorSpeed;
};


class SkidSteering
{
//variables
public:
protected:
private:
	uint8_t throttleLeft;
	uint8_t throttleRight;
	uint8_t steeringOffsetFromCentre;
	
	bool directionIsForward;
	bool directionIsForwardForLeftMotor;
	bool directionIsForwardForRightMotor;
	bool brakesAreOn;

	MotorPinDefinition leftMotorPinDef, rightMotorPinDef;
	
//functions
public:
	SkidSteering(MotorPinDefinition leftMotor, MotorPinDefinition rightMotor);
	~SkidSteering();
	void processInputs(short rawThrottle, short rawSteering, short rawDirection);
	
protected:
private:
	enum TURN_DIRECTION {LEFT, AHEAD, RIGHT};
		
	SkidSteering( const SkidSteering &c );
	SkidSteering& operator=( const SkidSteering &c );
	
	void setupMotorShield();
	
	void handleTurning(
		TURN_DIRECTION direction,
		uint8_t throttle,
		uint8_t steering,
		uint8_t steeringOffsetFromCentre,
		uint8_t *motorThrottleLeft,
		uint8_t *motorThrottleRight);

	void handleAhead(
	uint8_t throttle,
	uint8_t steering,
	uint8_t steeringOffsetFromCentre,
	uint8_t *motorThrottleLeft,
	uint8_t *motorThrottleRight);
	
	void handleTurningLeft(
		uint8_t throttle,
		uint8_t steering,
		uint8_t steeringOffsetFromCentre,
		uint8_t *motorThrottleLeft,
		uint8_t *motorThrottleRight);
		
	void handleTurningRight(
		uint8_t throttle,
		uint8_t steering,
		uint8_t steeringOffsetFromCentre,
		uint8_t *motorThrottleLeft,
		uint8_t *motorThrottleRight);
		
	float getMilliAmpsPerMotor(uint8_t motorPin);
	
	void setBothMotorBrakesOn();
	
	void setBothMotorBrakesOff();
	
	void setMotorBrake(uint8_t pin, boolean on);
	
	void setDirectionOfBothMotorsToForward();
	
	void setDirectionOfBothMotorsToReverse();
	
	void setDirectionOfMotorToForward(uint8_t pin, boolean forward);
	
	void setBothMotorsSpeed(short value);
	
	void setMotorSpeed(uint8_t pin, short value);
	
}; //SkidSteering

#endif //__SKIDSTEERING_H__
