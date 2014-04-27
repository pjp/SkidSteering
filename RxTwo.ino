#include "DisplayValueOnLed.h"
#include "SkidSteering.h"

//////////////////////////////
//The throttle and rudder pins
#define THROTTLE			4	// Digital pin
#define STEERING			5	// Digital pin
#define DIRECTION			6	// Digital pin

#define LED_PIN				7	// Digital pin

#define SUPPLY_VOLTAGE_PIN	2	// Analogue pin

DisplayValueOnLed			dvol(LED_PIN, 0, 1, 8);	// For using the LED to display battery state

SkidSteering				*skidSteering;

void setup()
{
	#if defined(VM_DEBUG)
		Serial.begin(115200);
	#endif

	struct MotorPinDefinition leftMotor;
	struct MotorPinDefinition rightMotor;

	///////////////////
	// Define motor pins
	leftMotor.motorAmps			=	0;			
	leftMotor.motorBrake		=	9;
	leftMotor.motorDirection	=	12;
	leftMotor.motorSpeed		=	3;
	
	rightMotor.motorAmps		=	1;
	rightMotor.motorBrake		=	8;
	rightMotor.motorDirection	=	13;
	rightMotor.motorSpeed		=	11;
	
	skidSteering	= new SkidSteering(leftMotor, rightMotor);
	
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
	skidSteering->processInputs(throttleIn, steeringIn, directionIn);
	
	///////////////////
	// Handle LED state
	updateLedValue();
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


