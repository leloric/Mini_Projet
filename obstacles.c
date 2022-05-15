#include <stdlib.h>
#include <stdint.h>

#include <obstacles.h>
#include <motor.h>
#include <audio_processing.h>

#include <motors.h>
#include <sensors/proximity.h>

#define HIGH_PROX		    	150
#define LOW_PROX				50
#define SECURITY				30
#define RATIO					90
#define IR1						0
#define IR3						2
#define IR6						5
#define IR8						7
#define OFF					    0


//this function will move the robot while avoiding the obstacles thanks to the IR detectors
void detection(void) {
	//Obstacle (front) and no obstacles on the left+right
	if((get_calibrated_prox(IR8)>HIGH_PROX || get_calibrated_prox(IR1)>HIGH_PROX) &&
		get_calibrated_prox(IR3)<LOW_PROX && get_calibrated_prox(IR6)<LOW_PROX) {
		turn_right();
		set_angle_found(OFF);
	}
	//Obstacle (front+right) and no obstacle on the left
	else if((get_calibrated_prox(IR8)>HIGH_PROX || get_calibrated_prox(IR1)>HIGH_PROX) &&
			get_calibrated_prox(IR3)>LOW_PROX && get_calibrated_prox(IR6)<LOW_PROX) {
			turn_left();
			set_angle_found(OFF);
	}
	//Obstacle (front+left) and no obstacle on the right
	else if((get_calibrated_prox(IR8)>HIGH_PROX || get_calibrated_prox(IR1)>HIGH_PROX) &&
			get_calibrated_prox(IR6)>LOW_PROX && get_calibrated_prox(IR3)<LOW_PROX) {
			turn_right();
			set_angle_found(OFF);
			}
	//Obstacle (right+left) and no obstacle up front
	else if((get_calibrated_prox(IR8)<HIGH_PROX || get_calibrated_prox(IR1)<HIGH_PROX) &&
			get_calibrated_prox(IR6)>LOW_PROX && get_calibrated_prox(IR3)>LOW_PROX) {

			//getting to close to the right obstacle
			if((get_calibrated_prox(IR8)<SECURITY || get_calibrated_prox(IR1)<SECURITY) &&
				(get_calibrated_prox(IR3)-get_calibrated_prox(IR6))>RATIO) {
				left_step();
				set_angle_found(OFF);

			//getting to close to the left obstacle
			} else if((get_calibrated_prox(IR8)<SECURITY || get_calibrated_prox(IR1)<SECURITY) &&
					 (get_calibrated_prox(IR6)-get_calibrated_prox(IR3))>RATIO) {
					right_step();
					set_angle_found(OFF);
			}
	}
}






