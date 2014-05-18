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

#define FULL_PULSE_WIDTH			(MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
#define HALF_PULSE_WIDTH			(MIN_PULSE_WIDTH + (FULL_PULSE_WIDTH / 2))
#define QUARTER_PULSE_WIDTH			(MIN_PULSE_WIDTH + (FULL_PULSE_WIDTH / 4))
#define THREE_QUARTERS_PULSE_WIDTH	(MIN_PULSE_WIDTH + (3 * (FULL_PULSE_WIDTH / 4)))

#define MIN_STARTUP_COUNT	20

struct MotorPinDefinition {
	uint8_t motorAmps;
	uint8_t motorBrake;
	uint8_t motorDirection;
	uint8_t motorSpeed;
};

struct SteeringConfig {
	uint8_t deadZone;
	int directionChangeDelay;
	float voltsPerAmp;
	float voltsPerBit;
};

enum HEADING		{STOPPED, FORWARD, BACKWARD};
enum TURN_DIRECTION {LEFT, STRAIGHT, RIGHT};

class SkidSteering
{
//variables
public:
protected:
	bool atStartup;	// Indicate we are at startup 
	int startupCount;
	
private:
	uint8_t throttleLeft;
	uint8_t throttleRight;
	uint8_t steeringOffsetFromCentre;
	
	bool generalDirectionIsForward;
	bool directionIsForwardForLeftMotor;
	bool directionIsForwardForRightMotor;
	bool brakesAreOn;
	bool weAreStopped;
	
	SteeringConfig steeringConfig;

	MotorPinDefinition leftMotorPinDef, rightMotorPinDef;
	
	
//functions
public:
	SkidSteering(SteeringConfig config, MotorPinDefinition leftMotor, MotorPinDefinition rightMotor);
	
	~SkidSteering();
	
	void reset();
	
	String processInputs(short throttle, short steering);


private:

	SkidSteering( const SkidSteering &c );
	
	SkidSteering& operator=( const SkidSteering &c );
	
protected:
	
	void setupMotorShield();
	
	String ftos(float value, int digitCount, int decimalPointsCount);
	
	void handleTurning(
		TURN_DIRECTION direction,
		HEADING heading,
		short throttleOffsetFromCentre,
		short steeringOffsetFromCentre,
		uint8_t *motorThrottleLeft,
		uint8_t *motorThrottleRight);

	void handleStraightAhead(
		HEADING heading,
		short throttleOffsetFromCentre,
		short steeringOffsetFromCentre,
		uint8_t *motorThrottleLeft,
		uint8_t *motorThrottleRight);
	
	void handleTurningLeft(
		HEADING heading,
		short throttleOffsetFromCentre,
		short steeringOffsetFromCentre,
		uint8_t *motorThrottleLeft,
		uint8_t *motorThrottleRight);
		
	void handleTurningRight(
		HEADING heading,
		short throttleOffsetFromCentre,
		short steeringOffsetFromCentre,
		uint8_t *motorThrottleLeft,
		uint8_t *motorThrottleRight);
		
	float getMilliAmpsPerMotor(uint8_t motorPin);
	
	void setBothMotorBrakesOff();
	
	void setMotorBrake(uint8_t pin, boolean on);
	
	void syncDirectionOfBothMotorsToGeneralDirection();
	
	void setDirectionOfBothMotorsToForward();
	
	void setDirectionOfBothMotorsToReverse();
	
	void setDirectionOfMotorToForward(uint8_t pin, boolean forward);
	
	void setBothMotorBrakesOn();
	
	void setBothMotorsSpeed(short value);
	
	void setMotorSpeed(uint8_t pin, short value);
	
	bool inDeadZone(int value);
	
}; //SkidSteering

#endif //__SKIDSTEERING_H__
