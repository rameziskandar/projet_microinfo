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

#define ROT_SPEED		500
#define AVANCE_SPEED	600
#define SCAN_ANGLE		PI/9	//angle in rad
#define TIME_LEFT		240		//time to go SCAN_ANGLE to the left
#define TIME_RIGHT		480		//time to go SCAN_ANGLE to the right
#define SPEED_FORWARD	77.28	//[mm/s] for ROT_SPEED = 600 [steps/s]
#define TURN_SPEED		2.43	//[rad/s] for ROT_SPEED = 500 [steps/s]
#define AVOID_MARGIN	2.5

#define NO_OBSTACLE		500  	//minimum threshold for when there is no obstacle scanned



void avoid_obstacle(uint16_t distance){

	struct vector {
		float x;
		float y;
	};

	static struct vector vect_left;
	static struct vector vect_right;
	static struct vector vect_diff;

	static uint16_t distance_left;
	static uint16_t distance_right;

	static float beta;
	static uint16_t avoid_time;
	static float straight_time;
	static float norm;


	turn_left(TIME_LEFT);
	distance_left = VL53L0X_get_dist_mm();

	//vecteur du scan a gauche
	vect_left.x = -(distance_left * sin(SCAN_ANGLE));
	vect_left.y = distance_left * cos(SCAN_ANGLE);

	turn_right(TIME_RIGHT);
	distance_right = VL53L0X_get_dist_mm();

	//vecteur du scan a droite
	vect_right.x = distance_right * sin(SCAN_ANGLE);
	vect_right.y = distance_right * cos(SCAN_ANGLE);


	if(distance_right > NO_OBSTACLE){

		vect_diff.x = -vect_left.x;
		vect_diff.y = (-vect_left.y) + distance;
		beta = atan2f(vect_diff.x, vect_diff.y);

		avoid_time = convert_angle(fabs(beta) - SCAN_ANGLE);
		turn_right(avoid_time);

		norm = vector_magnitude(vect_diff.x, vect_diff.y);
		straight_time = convert_distance(AVOID_MARGIN * norm);
		go_straight(straight_time);
	}

	else if(distance_left > NO_OBSTACLE){

		vect_diff.x = vect_right.x;
		vect_diff.y = (-distance) + vect_right.y;
		beta = atan2f(vect_diff.x, vect_diff.y);

		avoid_time = convert_angle(PI-(fabs(beta)) + SCAN_ANGLE);
		turn_right(avoid_time);

		norm = vector_magnitude(vect_diff.x, vect_diff.y);
		straight_time = convert_distance(AVOID_MARGIN * norm);
		go_straight(straight_time);

	}

	else if(distance_left <= distance_right){

		vect_diff.x = (-vect_left.x) + vect_right.x;
		vect_diff.y = (-vect_left.y) + vect_right.y;
		beta = atan2f(vect_diff.x, vect_diff.y);

		avoid_time = convert_angle(fabs(beta) - SCAN_ANGLE);
		turn_right(avoid_time);

		norm = vector_magnitude(vect_diff.x, vect_diff.y);
		straight_time = convert_distance(AVOID_MARGIN * norm);
		go_straight(straight_time);
	}

	else if(distance_left > distance_right){

		vect_diff.x = (-vect_left.x) + vect_right.x;
		vect_diff.y = (-vect_left.y) + vect_right.y;
		beta = atan2f(vect_diff.x, vect_diff.y);

		avoid_time = convert_angle(PI-(fabs(beta)) + SCAN_ANGLE);
		turn_left(avoid_time);

		norm = vector_magnitude(vect_diff.x, vect_diff.y);
		straight_time = convert_distance(AVOID_MARGIN * norm);
		go_straight(straight_time);
	}
}


void turn_right(uint16_t turn_time){

	static systime_t time_start;
	time_start = chVTGetSystemTime();

	while(chVTGetSystemTime()-time_start < turn_time){
		left_motor_set_speed(ROT_SPEED);
		right_motor_set_speed(-ROT_SPEED);
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}


void turn_left(uint16_t turn_time){

	static systime_t time_start;
	time_start = chVTGetSystemTime();

	while(chVTGetSystemTime()-time_start < turn_time){
		left_motor_set_speed(-ROT_SPEED);
		right_motor_set_speed(ROT_SPEED);
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void go_straight(uint16_t time_forward){

	static systime_t time_start;
	time_start = chVTGetSystemTime();

	while(chVTGetSystemTime()-time_start < time_forward){
		left_motor_set_speed(AVANCE_SPEED);
		right_motor_set_speed(AVANCE_SPEED);
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

//converts an angle into a time the robot needs to turn to obtain that angle
uint16_t convert_angle(float angle){

	uint16_t turn_time;

	turn_time = 1000*fabs(angle)/TURN_SPEED;

	return turn_time;
}

//converts a distance into a time
uint16_t convert_distance(int16_t distance){

	uint16_t time_forward;

	time_forward = 1000*distance / SPEED_FORWARD;

	return time_forward;
}

//computes the norm of a vector
float vector_magnitude(float component_x, float component_y){

	float magnitude;

	magnitude = sqrtf(component_x*component_x + component_y*component_y);

	return magnitude;
}
