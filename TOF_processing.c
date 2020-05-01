/*
 * TOF_processing.c
 *
 *  Created on: 29 avr. 2020
 *      Author: Hendrik
 *
 */
#include "ch.h"
#include "hal.h"
#include <main.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <communications.h>
#include <arm_math.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <TOF_processing.h>

#define SCAN_SPEED		300
#define SCAN_ANGLE		30 //angle in degrees
#define DEGREES_LEFT	20 //number of "steps" to go SCAN_ANGLE to the left
#define DEGREES_RIGHT	40 //number of "steps" to go SCAN_ANGLE to the right
#define STEPS_FORWARD	50 //number of "steps" to parcour DIST_FORWARD
#define DIST_FORWARD	10 //distance in cm

#define DIST_STOP   	70  //distance between robot and obstacle in mm


void sensor_distance(void){
	static uint16_t distance;

	distance = VL53L0X_get_dist_mm();

	if(distance < DIST_STOP){

		avoid_obstacle();

	}

}

void avoid_obstacle(void){

	static uint16_t distance_left;
	static uint16_t distance_right;

	static int16_t vect_left_x;
	static int16_t vect_left_y;
	static int16_t vect_right_x;
	static int16_t vect_right_y;
	static int16_t vect_diff_x;
	static int16_t vect_diff_y;
	static int16_t beta;
	static int16_t avoid_angle;
	static int16_t avoid_distance;


	turn_left(DEGREES_LEFT);
	distance_left = VL53L0X_get_dist_mm();
	vect_left_x = -(distance_left * sin(SCAN_ANGLE));
	vect_left_y = distance_left * cos(SCAN_ANGLE);

	turn_right(DEGREES_RIGHT);
	distance_right = VL53L0X_get_dist_mm();
	vect_right_x = distance_right * sin(SCAN_ANGLE);
	vect_right_y = distance_right * cos(SCAN_ANGLE);

	vect_diff_x = (-vect_left_x) + vect_right_x;
	vect_diff_y = (-vect_left_y) + vect_right_y;

	beta = atan2(vect_diff_x, vect_diff_y);
	//il faudra convertir l'angle beta pour l'utiliser dans les
	// fonctions turn right et turn left
	avoid_angle = convert_angle(abs(beta) - SCAN_ANGLE);

	if(distance_left < distance_right){
		turn_right(avoid_angle);
		avoid_distance = 2 * convert_distance(distance_right);
		go_straight(avoid_distance);
	}
	else if(distance_left > distance_right){
		turn_left(avoid_angle);
		avoid_distance = 2 * convert_distance(distance_left);
		go_straight(avoid_distance);
	}

}

//void turn_right(uint32_t angle){
//	for(uint32_t i = 0; i < angle_right; i++){
//		left_motor_set_speed(SCAN_SPEED);
//		right_motor_set_speed(-SCAN_SPEED);
//	}
//	left_motor_set_speed(0);
//	right_motor_set_speed(0);
//}
void turn_right(uint32_t angle){
	systime_t time_start = chVTGetSystemTime();
	while(chVTGetSystemTime()-time_start < angle){
		left_motor_set_speed(SCAN_SPEED);
		right_motor_set_speed(-SCAN_SPEED);
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}


void turn_left(uint32_t angle){
	for(uint32_t i = 0; i < angle; i++){
		left_motor_set_speed(-SCAN_SPEED);
		right_motor_set_speed(SCAN_SPEED);
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void go_straight(uint32_t distance_forward){
	for(uint32_t i = 0; i < distance_forward; i++){
		left_motor_set_speed(SCAN_SPEED);
		right_motor_set_speed(SCAN_SPEED);
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

uint32_t convert_angle(int16_t angle){

	uint32_t steps;

	steps = (angle * DEGREES_LEFT)/SCAN_ANGLE;

	return steps;
}

uint32_t convert_distance(int16_t distance){

	uint32_t steps;

	steps = (distance * STEPS_FORWARD)/DIST_FORWARD;

	return steps;
}
