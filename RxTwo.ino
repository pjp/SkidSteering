#include "motor.h"
#include "DisplayValueOnLed.h"

//////////////////////////////
//The throttle and rudder pins
#define THROTTLE			4	// Digital pin
#define STEERING			5	// Digital pin
#define DIRECTION			6	// Digital pin

#define LED_PIN				7	// Digital pin

/////////////////////////////
// Battery voltage input pins
#define MOTOR_LEFT_PIN_AMPS	0	// Voltage proportional to Amps drawn
#define MOTOR_RIGHT_PIN_AMPS	1	// Voltage proportional to Amps drawn

#define SUPPLY_VOLTAGE_PIN	2	// Analogue pin

#define	MIN_PULSE_WIDTH		1000
#define	MAX_PULSE_WIDTH		2000
#define	WAIT_FOR_PULSE		20000
#define	FULL_RANGE_INPUT	255

#define TICK_DELAY_MS		250		// How long (in mS) to delay in each loop
#define DIRECTION_CHANGE_DELAY_MS	500		// How long (in mS) to delay after a direction change	

#define VOLTS_PER_AMP		1.65

#define HALF_RANGE_INPUT	(FULL_RANGE_INPUT / 2)
#define CENTRE_INPUT		HALF_RANGE_INPUT
#define DEAD_ZONE			(FULL_RANGE_INPUT / 50)

#define FULL_PULSE_WIDTH			(MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
#define HALF_PULSE_WIDTH			(MIN_PULSE_WIDTH + (FULL_PULSE_WIDTH / 2))
#define QUARTER_PULSE_WIDTH			(MIN_PULSE_WIDTH + (FULL_PULSE_WIDTH / 4))
#define THREE_QUARTERS_PULSE_WIDTH	(MIN_PULSE_WIDTH + (3 * (FULL_PULSE_WIDTH / 4)))

/////////////////
// For the motors
#define DIRECTION_BRAKE_ON	0
#define DIRECTION_FORWARD	1
#define DIRECTION_REVERSE	2

#define MOTOR_LEFT_BRAKE_PIN	9
#define MOTOR_RIGHT_BRAKE_PIN	8

#define MOTOR_LEFT_DIRECTION_PIN	12
#define MOTOR_RIGHT_DIRECTION_PIN	13

#define MOTOR_LEFT_SPEED_PIN		3
#define MOTOR_RIGHT_SPEED_PIN		11

const uint8_t REFERENCE_VOLTS	=	5;   // the default reference on a 5-volt board
const float VOLTS_PER_BIT		=	REFERENCE_VOLTS / 1024.0;
const float RESISTOR_FACTOR		=	1024.0 / 2;

const int MIN_MILLI_VOLTS			=	6600;	// 6 * 1.1
const int MAX_MILLI_VOLTS			=	8400;	// 6 * 1.4

bool directionIsForward				= true;	// General direction of travel.

bool directionIsForwardForLeftMotor		= directionIsForward;	// For possible spinning on the spot
bool directionIsForwardForRightMotor	= directionIsForward;	// For spinning possible on the spot

bool brakesAreOn					= true;	// Motor brakes applied.

DisplayValueOnLed			dvol(LED_PIN, 0, 1, 8);	// For using the LED to display battery state

void setup()
{
	#if defined(VM_DEBUG)
		Serial.begin(115200);
	#endif

	setupMotorShield();
	
	setupRxInput();
}


void loop()
{
	delay(TICK_DELAY_MS);	// Very important
	
	//////////////////////////////////////////
	// Read the pulses on the pins from the RX
	short throttleIn	= pulseIn(THROTTLE,		HIGH, WAIT_FOR_PULSE);
	short steeringIn	= pulseIn(STEERING,		HIGH, WAIT_FOR_PULSE);
	short directionIn	= pulseIn(DIRECTION,	HIGH, WAIT_FOR_PULSE);
	
	/////////////////////
	// Process the inputs
	processInputs(throttleIn, steeringIn, directionIn);
	
	///////////////////
	// Handle LED state
	updateLedValue();
}

void processInputs(short rawThrottle, short rawSteering, short rawDirection) {
	/////////////////////////////
	// Check if the radios are on
	if(rawThrottle < (MIN_PULSE_WIDTH / 2)) {
		// Receiver or transmitter not on
		#if defined(VM_DEBUG)
		Serial.println("Radios not on");
		#endif
		
		return;
	}

	///////////////////////
	// Normalize the inputs
	uint8_t throttle	= map(rawThrottle	< MIN_PULSE_WIDTH ? MIN_PULSE_WIDTH : rawThrottle,	MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, FULL_RANGE_INPUT);
	uint8_t steering	= map(rawSteering	< MIN_PULSE_WIDTH ? MIN_PULSE_WIDTH : rawSteering,	MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, FULL_RANGE_INPUT);
	
	/////////////////////////////////////////////
	// Figure out the 3 position direction switch
	if(rawDirection < QUARTER_PULSE_WIDTH) {
		// Apply the brakes if not already on
		if(! brakesAreOn) {
			setBothMotorBrakesOn();
			
			delay(DIRECTION_CHANGE_DELAY_MS);
		}
	} else if(rawDirection > THREE_QUARTERS_PULSE_WIDTH) {
		// Set the direction to reverse if not already so
		if(directionIsForward) {
			setDirectionOfBothMotorsToReverse();
			setBothMotorBrakesOff();
			
			delay(DIRECTION_CHANGE_DELAY_MS);
		}
	} else {
		// Set the direction to forward if not already so
		if(! directionIsForward) {
			setDirectionOfBothMotorsToForward();
			setBothMotorBrakesOff();
			
			delay(DIRECTION_CHANGE_DELAY_MS);
		}
	}

	uint8_t throttleLeft					= 0;
	uint8_t throttleRight					= 0;
	uint8_t steeringOffsetFromCentre		= abs(HALF_RANGE_INPUT - steering);
	
	//////////////////////////////////////////////////////////////
	// Work out the relative throttle values with steering applied
	if(steering < (HALF_RANGE_INPUT - DEAD_ZONE)) {
		handleTurning(LEFT, throttle, steering, steeringOffsetFromCentre, &throttleLeft, &throttleRight);
		
	} else if(steering > (HALF_RANGE_INPUT + DEAD_ZONE)) {
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
		setMotorSpeed(MOTOR_LEFT_SPEED_PIN, throttleLeft);
		setMotorSpeed(MOTOR_RIGHT_SPEED_PIN, throttleRight);
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
// Utility functions ////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
void handleTurning(
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

void handleAhead(
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

void handleTurningLeft(
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
					setDirectionOfMotorToForward(MOTOR_LEFT_DIRECTION_PIN, true);
				} else {
					setDirectionOfMotorToForward(MOTOR_LEFT_DIRECTION_PIN, false);
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
					setDirectionOfMotorToForward(MOTOR_LEFT_DIRECTION_PIN, false);
				} else {
					setDirectionOfMotorToForward(MOTOR_LEFT_DIRECTION_PIN, true);
				}
			}
		}
	}
}

void handleTurningRight(
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
					setDirectionOfMotorToForward(MOTOR_RIGHT_DIRECTION_PIN, true);
				} else {
					setDirectionOfMotorToForward(MOTOR_RIGHT_DIRECTION_PIN, false);
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
					setDirectionOfMotorToForward(MOTOR_RIGHT_DIRECTION_PIN, false);
				} else {
					setDirectionOfMotorToForward(MOTOR_RIGHT_DIRECTION_PIN, true);
				}
			}
		}
	}
}


void setupMotorShield() {
	//////////////////
	// Motor Shield
	//
	// Setup Channel A - LEFT
	pinMode(MOTOR_LEFT_DIRECTION_PIN,	OUTPUT);	// Motor pin
	pinMode(MOTOR_LEFT_BRAKE_PIN,		OUTPUT);	// Brake pin
	
	// Setup Channel B - RIGHT
	pinMode(MOTOR_RIGHT_DIRECTION_PIN,	OUTPUT);	// Motor pin
	pinMode(MOTOR_RIGHT_BRAKE_PIN,		OUTPUT);	// Brake pin
	
	/////////////////////////////
	// Initially apply the brakes
	setBothMotorBrakesOn();
}

void setupRxInput() {
	pinMode(THROTTLE,	INPUT);
	pinMode(STEERING,	INPUT);
	pinMode(DIRECTION,	INPUT);
}

float getSupplyVoltage() {
	//////////////////////////
	// Read the voltage supply
	short voltageIn		= analogRead(SUPPLY_VOLTAGE_PIN);  // read the value from the sensor

	float supplyVolts	= (voltageIn / RESISTOR_FACTOR) * REFERENCE_VOLTS ; // calculate the ratio
	
	return supplyVolts;
}

float getMilliAmpsPerMotor(uint8_t motorPin) {
	/////////////////////////////////////
	// Calculate amps drawn by the motors

	int   voltsMotor		= analogRead(motorPin) ;
	
	float volts				= voltsMotor * VOLTS_PER_BIT;
	
	float milliAmpsForMotor	= (volts * 1000) / VOLTS_PER_AMP;  // read the value from the sensor
	
	return milliAmpsForMotor;
}

void setBothMotorBrakesOn() {
	setMotorBrake(MOTOR_LEFT_BRAKE_PIN, true);
	setMotorBrake(MOTOR_RIGHT_BRAKE_PIN, true);
	
	brakesAreOn	=	true;
}

void setBothMotorBrakesOff() {
	setMotorBrake(MOTOR_LEFT_BRAKE_PIN, false);
	setMotorBrake(MOTOR_RIGHT_BRAKE_PIN, false);
	
	brakesAreOn	=	false;
}

void setMotorBrake(uint8_t pin, boolean on) {
	if(on) {
		digitalWrite(pin, HIGH);
		} else {
		digitalWrite(pin, LOW);
	}
}

void setDirectionOfBothMotorsToForward() {
	setDirectionOfMotorToForward(MOTOR_LEFT_DIRECTION_PIN, true);
	setDirectionOfMotorToForward(MOTOR_RIGHT_DIRECTION_PIN, true);
	
	directionIsForward	= true;
}

void setDirectionOfBothMotorsToReverse() {
	setDirectionOfMotorToForward(MOTOR_LEFT_DIRECTION_PIN, false);
	setDirectionOfMotorToForward(MOTOR_RIGHT_DIRECTION_PIN, false);

	directionIsForward	= false;
}

void setDirectionOfMotorToForward(uint8_t pin, boolean forward) {
	if(forward) {
		digitalWrite(pin, HIGH);
		
		if(pin == MOTOR_LEFT_DIRECTION_PIN) {
			directionIsForwardForLeftMotor	= true;		
		} else {
			directionIsForwardForRightMotor	= true;
		}
	} else {
		digitalWrite(pin, LOW);
		
		if(pin == MOTOR_LEFT_DIRECTION_PIN) {
			directionIsForwardForLeftMotor	= false;	
		} else {
			directionIsForwardForRightMotor	= false;
		}
	}
}

void setBothMotorsSpeed(short value) {
	setMotorSpeed(MOTOR_LEFT_SPEED_PIN, value);
	setMotorSpeed(MOTOR_RIGHT_SPEED_PIN, value);
}

void setMotorSpeed(uint8_t pin, short value) {
	analogWrite(pin, value);
}

void updateLedValue() {
	int milliVolts	= getSupplyVoltage() * 1000 ;
	
	if(milliVolts < MIN_MILLI_VOLTS) {
		milliVolts	=	MIN_MILLI_VOLTS;
	}
	
	/////////////////////////////////////////////////////////
	// Just need a range from 1..10 to indicate battery state
	int value		= map(milliVolts, MIN_MILLI_VOLTS, MAX_MILLI_VOLTS, 1, 10);
	
	dvol.tick(value);	// All the work is done in the tick() methods

}


