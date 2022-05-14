#include <stdlib.h>
#include <sensors/proximity.h>
#include <motor.h>
#include <audio/play_melody.h>
#include <motors.h>
#include <audio_processing.h>
#include "leds.h"

#define SIDE_OBSTACLE			3
#define FRONT_OBSTACLE			150
#define NO_OBSTACLE_SECURITY	15
#define NEED_CALIBRATION		90
#define FRONT_SECURITY			30
#define MIDDLE_CROSS			325
#define MIDDLE_CROSS_R			75
#define OUT_CROSS				650
#define RESET					0
#define SET						1
#define IR1						0
#define IR3						2
#define IR6						5
#define IR8						7
#define LA						440	//Hz
#define QUARTER_SECOND			250 //ms


/*
 * Calls the right motor function to move the robot, depending on its surroundings via IR detectors
 */

void detection(void) {
	//Obstacle (front) and no obstacles on the left+right
	if((get_calibrated_prox(IR8)>110 || get_calibrated_prox(IR1)>110) &&
		get_calibrated_prox(IR3)<110 && get_calibrated_prox(IR6)<110){
		turn_right();
	}
	//Obstacle (front+right) and no obstacle on the left
	else if((get_calibrated_prox(IR8)>200 || get_calibrated_prox(IR1)>200) &&
			get_calibrated_prox(IR3)>110 && get_calibrated_prox(IR6)<110) {
			set_led(LED7, 1);
			turn_left();
			}
	//Obstacle (front+left) and no obstacle on the right
	else if((get_calibrated_prox(IR8)>110 || get_calibrated_prox(IR1)>110) &&
			get_calibrated_prox(IR6)>110 && get_calibrated_prox(IR3)<110) {
			set_led(LED3, 1);
			turn_right();
			}
	//Obstacle (right+left) and no obstacle up front
	else if((get_calibrated_prox(IR8)<110 || get_calibrated_prox(IR1)<110) &&
			get_calibrated_prox(IR6)>110 && get_calibrated_prox(IR3)>110) {

			if((get_calibrated_prox(IR8)<30 || get_calibrated_prox(IR1)<30) &&
				(get_calibrated_prox(IR3)-get_calibrated_prox(IR6))>90) {
				left_step();
				}
			else if((get_calibrated_prox(IR8)<30 || get_calibrated_prox(IR1)<30) &&
					(get_calibrated_prox(IR6)-get_calibrated_prox(IR3))>90) {
					right_step();
					}
		}
		else {
			straight_track();
			}
}






