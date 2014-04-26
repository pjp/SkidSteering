/*
 * Motor.h
 *
 * Created: 2014-04-26 19:19:01
 *  Author: pjp
 */ 
#include "Arduino.h"

#ifndef MOTOR_H_
#define MOTOR_H_

enum TURN_DIRECTION {LEFT, AHEAD, RIGHT};
	
void handleTurning(
	TURN_DIRECTION direction,
	uint8_t throttle,
	uint8_t steering,
	uint8_t steeringOffsetFromCentre,
	uint8_t *motorThrottleLeft,
	uint8_t *motorThrottleRight);



#endif /* MOTOR_H_ */