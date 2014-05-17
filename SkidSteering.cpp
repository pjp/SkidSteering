/* 
* SkidSteering.cpp
*
* Created: 2014-04-26 22:22:56
* Author: pjp
*/


#include "SkidSteering.h"

//////////////////////
// default constructor
SkidSteering::SkidSteering(SteeringConfig config, MotorPinDefinition leftMotor, MotorPinDefinition rightMotor)
{
	steeringConfig						= config;
	
	leftMotorPinDef						= leftMotor;
	
	rightMotorPinDef					= rightMotor;
	
	reset();

	setupMotorShield();
} 

/////////////////////
// default destructor
SkidSteering::~SkidSteering()
{
} 

///////////////////////////
// (Re)Initialize variables
void SkidSteering::reset() {
	atStartup							= true;
	
	generalDirectionIsForward			= true;	// General direction of travel.

	directionIsForwardForLeftMotor		= generalDirectionIsForward;	// For possible spinning on the spot
	
	directionIsForwardForRightMotor		= generalDirectionIsForward;	// For possible spinning on the spot

	weAreStopped						= true;	// Not moving
	
	setBothMotorBrakesOn();
}

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
/* @param throttle  0 = full reverse, 127 = stopped, 255 full forward   */
/* @param steering  0 = full left, 127 = ahead, 255 full right          */
/* @return A String containing the state of the variables               */
/************************************************************************/
String SkidSteering::processInputs(short throttle, short steering) {
	String state	=	"";
	
	state += " Su:";
	state.concat(atStartup);

	state += " T:";
	state.concat(throttle);
	
	state += " S:";
	state.concat(steering);

	short  steeringOffsetFromCentre			= (steering - HALF_RANGE_INPUT);
	state += " So:";
	state += steeringOffsetFromCentre;

	short throttleOffsetFromCentre			= (throttle - HALF_RANGE_INPUT);
	state += " To:";
	state += throttleOffsetFromCentre;
	
	//////////////////////////////////////////////////////////////////////
	// Check to see if the throttle is set to halfway initially at startup
	short absThrottleOffsetFromCentre	=	abs(throttleOffsetFromCentre);
	
	if(atStartup && absThrottleOffsetFromCentre < steeringConfig.deadZone) {
		atStartup	= false;	// yes - we can start	
		
		setBothMotorBrakesOff();
	}
	
	if(atStartup) {
		// Throttle is not halfway indicating no movement so don't carry on
		return state;
	}
	
	//////////////////////////////////////////
	// Calculate the heading from the throttle
	HEADING heading;
	
	if(inDeadZone(absThrottleOffsetFromCentre)) {
		heading			=	STOPPED;
		state			+=	" Hd:S";
		weAreStopped	=	true;
		
	} else if(throttleOffsetFromCentre > steeringConfig.deadZone) {
		heading						=	FORWARD;
		generalDirectionIsForward	=	 true;
		
		syncDirectionOfBothMotorsToGeneralDirection();
		
		weAreStopped		=	false;
		
		state				+=	" Hd:F";
		
	} else {
		heading						=	BACKWARD;
		generalDirectionIsForward	=	 false;
		
		syncDirectionOfBothMotorsToGeneralDirection();

		weAreStopped		=	false;
		
		state				+=	" Hd:B";
		
	}
	
	//////////////////////////////////////////////////////////////
	// Work out the relative throttle values with steering applied
	uint8_t throttleLeft					= 0;
	uint8_t throttleRight					= 0;

	state += " Ht:";
	
	int absSteeringOffsetFromCentre	=	abs(steeringOffsetFromCentre);
	
	if(inDeadZone(steeringOffsetFromCentre)) {
		handleTurning(STRAIGHT, heading, absThrottleOffsetFromCentre, absSteeringOffsetFromCentre, &throttleLeft, &throttleRight);
		
		state += "s";		
		
	} else if(steeringOffsetFromCentre > steeringConfig.deadZone) {
		handleTurning(RIGHT, heading, absThrottleOffsetFromCentre, absSteeringOffsetFromCentre, &throttleLeft, &throttleRight);
		
		state += "r";
		
	} else {
		handleTurning(LEFT, heading, absThrottleOffsetFromCentre, absSteeringOffsetFromCentre, &throttleLeft, &throttleRight);
		
		state += "l";
		
	}
	
	//////////////////////////////////////////////////
	// Sanity checks & convert offsets into full range
	throttleLeft	=	constrain(throttleLeft * 2, 0, FULL_RANGE_INPUT);
	throttleRight	=	constrain(throttleRight * 2, 0, FULL_RANGE_INPUT);
	
	state += " Tl:";
	state += throttleLeft;

	state += " Tr:";
	state += throttleRight;

	state += " Df:";
	state += generalDirectionIsForward;
	
	state += " Dfl:";
	state +=directionIsForwardForLeftMotor;
	
	state += " Dfr:";
	state += directionIsForwardForRightMotor;
	
	state += " Bo:";
	state += brakesAreOn;

	state += " mAl:";
	state.concat(ftos(getMilliAmpsPerMotor(leftMotorPinDef.motorAmps), 7, 2));
	
	state += " mAr:";
	state.concat(ftos(getMilliAmpsPerMotor(rightMotorPinDef.motorAmps), 7, 2));

	/////////////////////
	// Can we now move  ?
	if(! brakesAreOn) {
		///////////////////////////////////////////////////////////
		// Spin the motors - throttle values are offset from centre
		setMotorSpeed(leftMotorPinDef.motorSpeed,	throttleLeft);
		setMotorSpeed(rightMotorPinDef.motorSpeed,	throttleRight);
	}
	
	return state;
}

/************************************************************************/
/* Convert a float value to it's String representation                  */
/************************************************************************/
String SkidSteering::ftos(float value, int digitCount, int decimalPointsCount) {
	char tmp[digitCount + decimalPointsCount + 2];	
	
	String fval = dtostrf(value, digitCount, decimalPointsCount, tmp);
	
	return fval;
}

void SkidSteering::handleTurning(
	TURN_DIRECTION direction,
	HEADING heading,
	short throttleOffsetFromCentre,
	short steeringOffsetFromCentre,
	uint8_t *motorThrottleLeft,
	uint8_t *motorThrottleRight) {
	
	switch(direction) {
		case STRAIGHT:
			handleStraightAhead(heading, throttleOffsetFromCentre, steeringOffsetFromCentre, motorThrottleLeft, motorThrottleRight);
			break;
		case LEFT:
			handleTurningLeft(heading, throttleOffsetFromCentre, steeringOffsetFromCentre, motorThrottleLeft, motorThrottleRight);
			break;
		case RIGHT:
			handleTurningRight(heading, throttleOffsetFromCentre, steeringOffsetFromCentre, motorThrottleLeft, motorThrottleRight);
			break;
	}
}

void SkidSteering::handleStraightAhead(
	HEADING heading,
	short throttleOffsetFromCentre,
	short steeringOffsetFromCentre,
	uint8_t *motorThrottleLeft,
	uint8_t *motorThrottleRight) {
	
	*motorThrottleLeft	= throttleOffsetFromCentre;
	*motorThrottleRight	= throttleOffsetFromCentre;
}

void SkidSteering::handleTurningLeft(
	HEADING heading,
	short throttleOffsetFromCentre,
	short steeringOffsetFromCentre,
	uint8_t *motorThrottleLeft,
	uint8_t *motorThrottleRight) {

	if(weAreStopped) {
		//////////////////////////////////////////////////
		// Need to spin on the spot and reverse left track
		*motorThrottleLeft	=	steeringOffsetFromCentre;
		*motorThrottleRight	=	steeringOffsetFromCentre;
		
		if(generalDirectionIsForward) {
			setDirectionOfMotorToForward(leftMotorPinDef.motorDirection, false);
		} else {
			setDirectionOfMotorToForward(leftMotorPinDef.motorDirection, true);
		}
	} else {
		if(throttleOffsetFromCentre > steeringOffsetFromCentre) {
			*motorThrottleLeft	=	(throttleOffsetFromCentre - steeringOffsetFromCentre);	// Slow down the left track
			*motorThrottleRight	=	throttleOffsetFromCentre;
		} else {
			*motorThrottleLeft	=	throttleOffsetFromCentre;
			*motorThrottleRight	=	(throttleOffsetFromCentre + steeringOffsetFromCentre);	// Speed up the right track
		}
	}
}

void SkidSteering::handleTurningRight(
	HEADING heading,
	short throttleOffsetFromCentre,
	short steeringOffsetFromCentre,
	uint8_t *motorThrottleLeft,
	uint8_t *motorThrottleRight) {

	if(weAreStopped) {
		//////////////////////////////////////////////////
		// Need to spin on the spot and reverse right track
		*motorThrottleLeft	=	steeringOffsetFromCentre;
		*motorThrottleRight	=	steeringOffsetFromCentre;
		
		if(generalDirectionIsForward) {
			setDirectionOfMotorToForward(rightMotorPinDef.motorDirection, false);
		} else {
			setDirectionOfMotorToForward(rightMotorPinDef.motorDirection, true);
		}
	} else {
		if(throttleOffsetFromCentre > steeringOffsetFromCentre) {
			*motorThrottleRight	=	(throttleOffsetFromCentre - steeringOffsetFromCentre);	// Slow down the right track
			*motorThrottleLeft	=	throttleOffsetFromCentre;
		} else {
			*motorThrottleRight	=	throttleOffsetFromCentre;
			*motorThrottleLeft	=	(throttleOffsetFromCentre + steeringOffsetFromCentre);	// Speed up the left track
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

void SkidSteering::syncDirectionOfBothMotorsToGeneralDirection() {
	if(generalDirectionIsForward != directionIsForwardForLeftMotor) {
		setDirectionOfMotorToForward(leftMotorPinDef.motorDirection, generalDirectionIsForward);
	}
	
	if(generalDirectionIsForward != directionIsForwardForRightMotor) {
		setDirectionOfMotorToForward(rightMotorPinDef.motorDirection, generalDirectionIsForward);	
	}
}

void SkidSteering::setDirectionOfBothMotorsToForward() {
	if(! directionIsForwardForLeftMotor) {
		setDirectionOfMotorToForward(leftMotorPinDef.motorDirection, true);
	}
	
	if(! directionIsForwardForRightMotor) {
		setDirectionOfMotorToForward(rightMotorPinDef.motorDirection, true);
	}
	
	generalDirectionIsForward	= true;
}

void SkidSteering::setDirectionOfBothMotorsToReverse() {
	if(directionIsForwardForLeftMotor) {
		setDirectionOfMotorToForward(leftMotorPinDef.motorDirection, false);
	}
	
	if(directionIsForwardForRightMotor) {
		setDirectionOfMotorToForward(rightMotorPinDef.motorDirection, false);
	}
	
	generalDirectionIsForward	= false;
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

bool SkidSteering::inDeadZone(int value) {
	bool inZone	=	false;
	
	if(abs(value) <= steeringConfig.deadZone) {
		inZone	= true;	
	}
	
	return inZone;
}
