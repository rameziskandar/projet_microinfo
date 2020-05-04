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

#define SCAN_SPEED		500
#define AVANCE_SPEED	600
#define SCAN_ANGLE		PI/9	//angle in rad
#define ANGLE1			PI/2	//angle in rad
#define DEGREES_LEFT	240		//time to go SCAN_ANGLE to the left
#define DEGREES_RIGHT	480		//time to go SCAN_ANGLE to the right
#define SPEED_FORWARD	77.28	//[mm/s] for SCAN_SPEED = 600 [steps/s]
#define TURN_SPEED		2.43	//[rad/s] for SCAN_SPEED = 500 [steps/s]

#define DIST_STOP   	70  	//distance between robot and obstacle in mm


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

	static float vect_left_x;
	static float vect_left_y;
	static float vect_right_x;
	static float vect_right_y;
	static float vect_diff_x;
	static float vect_diff_y;
	static float beta;
	static uint32_t avoid_angle;
	static float avoid_distance;

	//DEGREES LEFT en [ms] -> system ticks
	//int16_t test_angle = convert_angle(SCAN_ANGLE);
	//turn_left(test_angle);

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

	beta = atan2f(vect_diff_x, vect_diff_y);
	//il faudra convertir l'angle beta pour l'utiliser dans les
	// fonctions turn right et turn left


	if(distance_left <= distance_right){

		avoid_angle = convert_angle(fabs(beta) - SCAN_ANGLE);
		turn_right(avoid_angle);
		avoid_distance = convert_distance(100);
		go_straight(avoid_distance);
	}
	else if(distance_left > distance_right){

		avoid_angle = convert_angle(PI-(fabs(beta)) + SCAN_ANGLE);
		turn_left(avoid_angle);
		avoid_distance = convert_distance(100);
		go_straight(avoid_distance);
	}


}


void turn_right(uint32_t turn_time){
	static systime_t time_start;
	time_start = chVTGetSystemTime();
	//chprintf((BaseSequentialStream *)&SD3, "time system = %d\n", turn_time);

	while(chVTGetSystemTime()-time_start < turn_time){

		//chprintf((BaseSequentialStream *)&SD3, "time = %f\n", chVTGetSystemTime()-time_start);
		left_motor_set_speed(SCAN_SPEED);
		right_motor_set_speed(-SCAN_SPEED);
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}


void turn_left(uint32_t turn_time){
	//chprintf((BaseSequentialStream *)&SD3, "time system = %d\n", turn_time);
	static systime_t time_start;
	time_start = chVTGetSystemTime();

	while(chVTGetSystemTime()-time_start < turn_time){
//		chprintf((BaseSequentialStream *)&SD3, "time system = %f\n", chVTGetSystemTime()-time_start);
//		chprintf((BaseSequentialStream *)&SD3, "time = %f\n", turn_time);
		left_motor_set_speed(-SCAN_SPEED);
		right_motor_set_speed(SCAN_SPEED);
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void go_straight(uint32_t time_forward){
	static systime_t time_start;
	time_start = chVTGetSystemTime();
	while(chVTGetSystemTime()-time_start < time_forward){

		left_motor_set_speed(AVANCE_SPEED);
		right_motor_set_speed(AVANCE_SPEED);
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

uint32_t convert_angle(float angle){

	uint32_t turn_time;
	//chprintf((BaseSequentialStream *)&SD3, "time = %f\n", 1000*fabs(angle)/TURN_SPEED);
	turn_time = 1000*fabs(angle)/TURN_SPEED;

	return turn_time;
}

//distance in mm
uint32_t convert_distance(int16_t distance){

	uint32_t time_forward;

	time_forward = 1000*distance / SPEED_FORWARD;

	return time_forward;
}
