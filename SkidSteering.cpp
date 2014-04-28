/* 
* SkidSteering.cpp
*
* Created: 2014-04-26 22:22:56
* Author: pjp
*/


#include "SkidSteering.h"

// default constructor
SkidSteering::SkidSteering(SteeringConfig config, MotorPinDefinition leftMotor, MotorPinDefinition rightMotor)
{
	directionIsForward					= true;	// General direction of travel.

	directionIsForwardForLeftMotor		= directionIsForward;	// For possible spinning on the spot
	
	directionIsForwardForRightMotor		= directionIsForward;	// For spinning possible on the spot

	brakesAreOn							= true;	// Motor brakes applied.
	
	steeringConfig						= config;
	
	leftMotorPinDef						= leftMotor;
	
	rightMotorPinDef					= rightMotor;
	
	setupMotorShield();
} 

// default destructor
SkidSteering::~SkidSteering()
{
} //~SkidSteering

void SkidSteering::setupMotorShield() {
	//////////////////
	// Motor Shield
	//
	// Setup Channel A - LEFT
	pinMode(leftMotorPinDef.motorDirection,	OUTPUT);		// Motor pin
	pinMode(leftMotorPinDef.motorBrake,		OUTPUT);		// Brake pin
	
	// Setup Channel B - RIGHT
	pinMode(rightMotorPinDef.motorDirection,	OUTPUT);	// Motor pin
	pinMode(rightMotorPinDef.motorBrake,		OUTPUT);	// Brake pin
	
	/////////////////////////////
	// Initially apply the brakes
	setBothMotorBrakesOn();
}

/************************************************************************/
/*                                                                      */
/* @param throttle  0-255                                               */
/* @param steering  0 = full left, 127 = ahead, 255 full right          */
/* @param heading   STOPPED, FORWARD, BACKWARD                          */
/*                                                                      */
/************************************************************************/
void SkidSteering::processInputs(uint8_t throttle, uint8_t steering, HEADING heading) {
	
	switch(heading) {
		case STOPPED:
			// Apply the brakes if not already on
			if(! brakesAreOn) {
				setBothMotorBrakesOn();
			
				delay(steeringConfig.directionChangeDelay);
			}

			break;
		case FORWARD:
			// Set the direction to forward if not already so
			if(! directionIsForward) {
				setDirectionOfBothMotorsToForward();
				setBothMotorBrakesOff();
			
				delay(steeringConfig.directionChangeDelay);
			}
			
			break;
		case BACKWARD:
			// Set the direction to reverse if not already so
			if(directionIsForward) {
				setDirectionOfBothMotorsToReverse();
				setBothMotorBrakesOff();
			
				delay(steeringConfig.directionChangeDelay);
			}
			
			break;
	}

	uint8_t throttleLeft					= 0;
	uint8_t throttleRight					= 0;
	uint8_t steeringOffsetFromCentre		= abs(HALF_RANGE_INPUT - steering);
	
	//////////////////////////////////////////////////////////////
	// Work out the relative throttle values with steering applied
	if(steering < (HALF_RANGE_INPUT - steeringConfig.deadZone)) {
		handleTurning(LEFT, throttle, steering, steeringOffsetFromCentre, &throttleLeft, &throttleRight);
		
	} else if(steering > (HALF_RANGE_INPUT + steeringConfig.deadZone)) {
		handleTurning(RIGHT, throttle, steering, steeringOffsetFromCentre, &throttleLeft, &throttleRight);
		
	} else {
		handleTurning(AHEAD, throttle, steering, steeringOffsetFromCentre, &throttleLeft, &throttleRight);
		
	}
	
	////////////////
	// Sanity checks
	throttleLeft	=	constrain(throttleLeft, 0, FULL_RANGE_INPUT);
	throttleRight	=	constrain(throttleRight, 0, FULL_RANGE_INPUT);
	
	#if defined(VM_DEBUG)
	///////////////
	// Debug output
	Serial.print("T:");
	Serial.print(throttle);
	
	Serial.print(" S:");
	Serial.print(steering);
	
	Serial.print(" So:");
	Serial.print(steeringOffsetFromCentre);
	
	Serial.print(" Tl:");
	Serial.print(throttleLeft);

	Serial.print(" Tr:");
	Serial.print(throttleRight);

	Serial.print(" Df:");
	Serial.print(directionIsForward);
	
	Serial.print(" Dfl:");
	Serial.print(directionIsForwardForLeftMotor);
	
	Serial.print(" Dfr:");
	Serial.print(directionIsForwardForRightMotor);
	
	Serial.print(" Bo:");
	Serial.print(brakesAreOn);

	float value	= getSupplyVoltage();
	Serial.print(" V:");
	Serial.print(value);

	value	= getMilliAmpsPerMotor(MOTOR_LEFT_PIN_AMPS);
	Serial.print(" mAl:");
	Serial.print(value);
	
	value	= getMilliAmpsPerMotor(MOTOR_RIGHT_PIN_AMPS);
	Serial.print(" mAr:");
	Serial.print(value);
	
	Serial.println();
	#endif
	
	/////////////////////
	// Can we now move  ?
	if(! brakesAreOn) {
		//////////////////
		// Spin the motors
		setMotorSpeed(leftMotorPinDef.motorSpeed,	throttleLeft);
		setMotorSpeed(rightMotorPinDef.motorSpeed,	throttleRight);
	}
}

void SkidSteering::handleTurning(
	TURN_DIRECTION direction,
	uint8_t throttle,
	uint8_t steering,
	uint8_t steeringOffsetFromCentre,
	uint8_t *motorThrottleLeft,
	uint8_t *motorThrottleRight) {
	
	switch(direction) {
		case AHEAD:
			handleAhead(throttle, steering, steeringOffsetFromCentre, motorThrottleLeft, motorThrottleRight);
			break;
		case LEFT:
			handleTurningLeft(throttle, steering, steeringOffsetFromCentre, motorThrottleLeft, motorThrottleRight);
			break;
		case RIGHT:
			handleTurningRight(throttle, steering, steeringOffsetFromCentre, motorThrottleLeft, motorThrottleRight);
			break;
	}
}

void SkidSteering::handleAhead(
	uint8_t throttle,
	uint8_t steering,
	uint8_t steeringOffsetFromCentre,
	uint8_t *motorThrottleLeft,
	uint8_t *motorThrottleRight) {
	
	if(throttle < 1) {
		/////////////////////////////////////////////////////////////////
		// Make sure the general direction is correct since one motor may
		// be in the opposite direction
		if(directionIsForward) {
			setDirectionOfBothMotorsToForward();
		} else {
			setDirectionOfBothMotorsToReverse();
		}
	}
	
	*motorThrottleLeft	= throttle;
	*motorThrottleRight	= throttle;
}

void SkidSteering::handleTurningLeft(
	uint8_t throttle,
	uint8_t steering,
	uint8_t steeringOffsetFromCentre,
	uint8_t *motorThrottleLeft,
	uint8_t *motorThrottleRight) {

	if(throttle > steeringOffsetFromCentre) {
		*motorThrottleLeft	=	throttle - steeringOffsetFromCentre;	// Slow down the left track
		*motorThrottleRight	=	throttle;
		
	} else {
		*motorThrottleRight	=	throttle + steeringOffsetFromCentre;	// Speed up the right track
		
		if(throttle > 0) {
			*motorThrottleLeft	=	throttle;

			if(directionIsForward != directionIsForwardForLeftMotor) {
				if(directionIsForward) {
					setDirectionOfMotorToForward(leftMotorPinDef.motorDirection, true);
				} else {
					setDirectionOfMotorToForward(leftMotorPinDef.motorDirection, false);
				}
			}
		} else {
			*motorThrottleLeft	=	2 * steeringOffsetFromCentre;

			///////////////////////////////////
			// No throttle, so spin on the spot
			//
			// Reverse the direction of this motor compared to the other and increase it's speed
			if(directionIsForward == directionIsForwardForLeftMotor) {
				if(directionIsForward) {
					setDirectionOfMotorToForward(leftMotorPinDef.motorDirection, false);
				} else {
					setDirectionOfMotorToForward(leftMotorPinDef.motorDirection, true);
				}
			}
		}
	}
}

void SkidSteering::handleTurningRight(
	uint8_t throttle,
	uint8_t steering,
	uint8_t steeringOffsetFromCentre,
	uint8_t *motorThrottleLeft,
	uint8_t *motorThrottleRight) {

	if(throttle > steeringOffsetFromCentre) {
		*motorThrottleLeft	=	throttle;
		*motorThrottleRight	=	throttle - steeringOffsetFromCentre;	// Slow down the right track
		
	} else {
		*motorThrottleLeft	=	throttle + steeringOffsetFromCentre;	// Speed up the left track
		
		if(throttle > 0) {
			*motorThrottleRight	=	throttle ;

			if(directionIsForward != directionIsForwardForRightMotor) {
				if(directionIsForward) {
					setDirectionOfMotorToForward(rightMotorPinDef.motorDirection, true);
				} else {
					setDirectionOfMotorToForward(rightMotorPinDef.motorDirection, false);
				}
			}
		} else {
			*motorThrottleRight	=	2 * steeringOffsetFromCentre;

			///////////////////////////////////
			// No throttle, so spin on the spot
			//
			// Reverse the direction of this motor compared to the other and increase it's speed
			if(directionIsForward == directionIsForwardForRightMotor) {
				if(directionIsForward) {
					setDirectionOfMotorToForward(rightMotorPinDef.motorDirection, false);
				} else {
					setDirectionOfMotorToForward(rightMotorPinDef.motorDirection, true);
				}
			}
		}
	}
}


float SkidSteering::getMilliAmpsPerMotor(uint8_t motorPin) {
	/////////////////////////////////////
	// Calculate amps drawn by the motors

	int   voltsMotor		= analogRead(motorPin) ;
	
	float volts				= voltsMotor * steeringConfig.voltsPerBit;
	
	float milliAmpsForMotor	= (volts * 1000) / steeringConfig.voltsPerAmp;  // read the value from the sensor
	
	return milliAmpsForMotor;
}

void SkidSteering::setBothMotorBrakesOn() {
	setMotorBrake(leftMotorPinDef.motorBrake, true);
	setMotorBrake(rightMotorPinDef.motorBrake, true);
	
	brakesAreOn	=	true;
}

void SkidSteering::setBothMotorBrakesOff() {
	setMotorBrake(leftMotorPinDef.motorBrake, false);
	setMotorBrake(rightMotorPinDef.motorBrake, false);
	
	brakesAreOn	=	false;
}

void SkidSteering::setMotorBrake(uint8_t pin, boolean on) {
	if(on) {
		digitalWrite(pin, HIGH);
	} else {
		digitalWrite(pin, LOW);
	}
}

void SkidSteering::setDirectionOfBothMotorsToForward() {
	setDirectionOfMotorToForward(leftMotorPinDef.motorDirection, true);
	setDirectionOfMotorToForward(rightMotorPinDef.motorDirection, true);
	
	directionIsForward	= true;
}

void SkidSteering::setDirectionOfBothMotorsToReverse() {
	setDirectionOfMotorToForward(leftMotorPinDef.motorDirection, false);
	setDirectionOfMotorToForward(rightMotorPinDef.motorDirection, false);

	directionIsForward	= false;
}

void SkidSteering::setDirectionOfMotorToForward(uint8_t pin, boolean forward) {
	if(forward) {
		digitalWrite(pin, HIGH);
		
		if(pin == leftMotorPinDef.motorDirection) {
			directionIsForwardForLeftMotor	= true;
		} else {
			directionIsForwardForRightMotor	= true;
		}
	} else {
		digitalWrite(pin, LOW);
		
		if(pin == leftMotorPinDef.motorDirection) {
			directionIsForwardForLeftMotor	= false;
		} else {
			directionIsForwardForRightMotor	= false;
		}
	}
}

void SkidSteering::setBothMotorsSpeed(short value) {
	setMotorSpeed(leftMotorPinDef.motorSpeed,	value);
	setMotorSpeed(rightMotorPinDef.motorSpeed,	value);
}

void SkidSteering::setMotorSpeed(uint8_t pin, short value) {
	analogWrite(pin, value);
}
