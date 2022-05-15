#include <stdlib.h>
#include <stdint.h>

#include <obstacles.h>
#include <motor.h>
#include <audio_processing.h>

#include <motors.h>
#include <leds.h>
#include <sensors/proximity.h>

#define IR1						0
#define IR3						2
#define IR6						5
#define IR8						7

/*
 * Calls the right motor function to move the robot, depending on its surroundings via IR detectors
 */

void detection(void) {
	//Obstacle (front) and no obstacles on the left+right
	if((get_calibrated_prox(IR8)>110 || get_calibrated_prox(IR1)>110) &&
		get_calibrated_prox(IR3)<110 && get_calibrated_prox(IR6)<110){
		set_led(LED3, 1);
		turn_right();
		set_angle_found(0);
	}
	//Obstacle (front+right) and no obstacle on the left
	else if((get_calibrated_prox(IR8)>200 || get_calibrated_prox(IR1)>200) &&
			get_calibrated_prox(IR3)>110 && get_calibrated_prox(IR6)<110) {
			set_led(LED7, 1);
			turn_left();
			set_angle_found(0);
			}
	//Obstacle (front+left) and no obstacle on the right
	else if((get_calibrated_prox(IR8)>110 || get_calibrated_prox(IR1)>110) &&
			get_calibrated_prox(IR6)>110 && get_calibrated_prox(IR3)<110) {
			set_led(LED3, 1);
			turn_right();
			set_angle_found(0);
			}
	//Obstacle (right+left) and no obstacle up front
	else if((get_calibrated_prox(IR8)<110 || get_calibrated_prox(IR1)<110) &&
			get_calibrated_prox(IR6)>110 && get_calibrated_prox(IR3)>110) {

			if((get_calibrated_prox(IR8)<30 || get_calibrated_prox(IR1)<30) &&
				(get_calibrated_prox(IR3)-get_calibrated_prox(IR6))>90) {
				set_led(LED7, 1);
				left_step();
				set_angle_found(0);
				}
			else if((get_calibrated_prox(IR8)<30 || get_calibrated_prox(IR1)<30) &&
					(get_calibrated_prox(IR6)-get_calibrated_prox(IR3))>90) {
					set_led(LED3, 1);
					right_step();
					set_angle_found(0);
					}
	}
}






