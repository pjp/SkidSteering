#include "DisplayValueOnLed.h"
#include "SkidSteering.h"

//////////////////////////////
//The throttle and rudder pins
#define THROTTLE			4	// Digital pin
#define STEERING			5	// Digital pin


#define SUPPLY_VOLTAGE_PIN	2	// Analogue pin
#define TICK_DELAY_MS		250				// How long (in mS) to delay in each loop
#define DIRECTION_CHANGE_DELAY_MS	500		// How long (in mS) to delay after a direction change

/////////////////////////////////////////////
// These depend on the specific receiver used
#define	MIN_PULSE_WIDTH		990
#define	MAX_PULSE_WIDTH		1990

#define	WAIT_FOR_PULSE		20000

const int MIN_MILLI_VOLTS		=	6600;	// 6 * 1.1
const int MAX_MILLI_VOLTS		=	8400;	// 6 * 1.4

const uint8_t REFERENCE_VOLTS	=	5;		// the default reference on a 5-volt board
const float VOLTS_PER_BIT		=	REFERENCE_VOLTS / 1024.0;
const float VOLTS_PER_AMP		=	1.65;
const float RESISTOR_FACTOR		=	1024.0 / 2;

const int DEAD_ZONE_RANGE		=	30;

/********************************************************************************
* @param ledPin								// The digital pin the led is on
* @param value								// The initial value to display as LED flashes
* @param ledOnCountInTicks					// How many ticks for the LED to remain on, off time is the same
* @param repeatDelayCountInTicks			// How many ticks between displaying the value
*/
#define LED_PIN				7	// Digital pin

DisplayValueOnLed			dvol(LED_PIN, 0, 1, 8);	// For using the LED to display battery state

SkidSteering				*skidSteering;
SteeringConfig				*config;

void setup()
{
	#if defined(VM_DEBUG)
		Serial.begin(115200);
	#endif

	struct SteeringConfig		config;
	struct MotorPinDefinition	leftMotor;
	struct MotorPinDefinition	rightMotor;

	//////////////////////////
	// Define steering configuration
	config.deadZone				= DEAD_ZONE_RANGE;
	config.directionChangeDelay	= DIRECTION_CHANGE_DELAY_MS;
	config.voltsPerAmp			= VOLTS_PER_AMP;
	config.voltsPerBit			= VOLTS_PER_BIT;
			
	
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
	
	//////////////////////////////////
	// Create the skid steering object 
	skidSteering	= new SkidSteering(config, leftMotor, rightMotor);
	
	setupRxInput();
}


void loop()
{
	delay(TICK_DELAY_MS);	// Very important
	
	//////////////////////////////////////////
	// Read the pulses on the pins from the RX
	unsigned long throttleIn	= pulseIn(THROTTLE,		HIGH, WAIT_FOR_PULSE);
	unsigned long steeringIn	= pulseIn(STEERING,		HIGH, WAIT_FOR_PULSE);
	
	/////////////////////////////
	// Check if the radios are on
	if(throttleIn < (MIN_PULSE_WIDTH / 2)) {
		
		/////////////////////////////////
		// Receiver or transmitter not on
		#if defined(VM_DEBUG)
			Serial.print("Radios not on");
		#endif
		
		///////////////////////
		// Reset internal state
		skidSteering->reset();

		///////////////////
		// Handle LED state
		updateLedValue();
		
		#if defined(VM_DEBUG)
			Serial.println("");
		#endif

		return;
	}

	
	/////////////////////////
	// Normalize these inputs
	short throttle	= map(throttleIn	< MIN_PULSE_WIDTH ? MIN_PULSE_WIDTH : throttleIn,	MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, FULL_RANGE_INPUT);
	short steering	= map(steeringIn	< MIN_PULSE_WIDTH ? MIN_PULSE_WIDTH : steeringIn,	MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, FULL_RANGE_INPUT);

	////////////////////
	// Drive the vehicle
	String state	=	skidSteering->processInputs(throttle, steering);
	
	///////////////////
	// Handle LED state
	updateLedValue();
	
	#if defined(VM_DEBUG)
		Serial.println(state);
	#endif
}


void setupRxInput() {
	pinMode(THROTTLE,	INPUT);
	pinMode(STEERING,	INPUT);
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

	#if defined(VM_DEBUG)
		Serial.print(" V:");
		Serial.print(milliVolts);
	#endif	
	
	/////////////////////////////////////////////////////////
	// Just need a range from 1..10 to indicate battery state
	int value		= map(milliVolts, MIN_MILLI_VOLTS, MAX_MILLI_VOLTS, 1, 10);
	
	dvol.tick(value);	// All the work is done in the tick() methods

}


