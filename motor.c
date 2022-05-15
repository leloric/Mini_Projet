#include <stdlib.h>
#include <stdint.h>
//#include <stm32f4xx.h>
//#include <gpio.h>
#include <motor.h>
#include <audio_processing.h>

#include <motors.h>
#include <leds.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>


#define SPEED_CONTROL       0
#define POSITION_CONTROL    1
#define MARCHE			    1
#define ARRET			    0
#define TARE			    0

#define ANGLE_MIN		1 // [deg]
#define PETITE_ATTENTE		300
#define ON		1
#define OFF		0
#define LOW_SPEED		200
#define MIDDLE_SPEED	300
#define HIGH_SPEED		500
#define STEP		500
#define ZERO		0
#define BOUCLE		7
#define STOP		0

#define IR3						2
#define IR6						5
/*
void straight_track(void){
	if (get_rescue()){
		left_motor_set_speed(HIGH_SPEED);
		right_motor_set_speed(HIGH_SPEED);
	}
}
*/
void turn (void){
	if (get_rescue()){
		set_body_led(ON);
		left_motor_set_speed(-HIGH_SPEED);
		right_motor_set_speed(HIGH_SPEED);
	}
}

void victim_found(void){
	if (get_rescue()){
		set_front_led(OFF);
		turn();
		playMelody(IMPOSSIBLE_MISSION, ML_SIMPLE_PLAY, NULL);
		int j=0;
		while(j < BOUCLE) {
			set_led(LED1, ON);
			chThdSleepMilliseconds(PETITE_ATTENTE);
			set_led(LED3, ON);
			chThdSleepMilliseconds(PETITE_ATTENTE);
			set_led(LED5, ON);
			chThdSleepMilliseconds(PETITE_ATTENTE);
			set_led(LED7, ON);
			chThdSleepMilliseconds(PETITE_ATTENTE);
			set_led(LED1, OFF);
			chThdSleepMilliseconds(PETITE_ATTENTE);
			set_led(LED3, OFF);
			chThdSleepMilliseconds(PETITE_ATTENTE);
			set_led(LED5, OFF);
			chThdSleepMilliseconds(PETITE_ATTENTE);
			set_led(LED7, OFF);
			++j;
		}
		set_body_led(OFF);
		set_front_led(ON);
		left_motor_set_speed(STOP);
		right_motor_set_speed(STOP);
	}
}

void rotate(int16_t angle){
	if (get_rescue()) {
		if (abs(angle) > ANGLE_MIN && get_angle_found()==0){
			set_body_led(1);
			set_front_led(0);
			left_motor_set_speed(-250);
			right_motor_set_speed(250);
		} else {
			set_front_led(1);
			set_body_led(0);
			set_angle_found(1);
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
	}
}

void turn_left(void){
	if (get_rescue()){
		right_motor_set_pos(ZERO);
		while (abs(right_motor_get_pos()) < 325){
			left_motor_set_speed(-HIGH_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		while (get_calibrated_prox(IR3) > 150){
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		right_motor_set_pos(ZERO);
		while (abs(right_motor_get_pos()) < 400){
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		right_motor_set_pos(ZERO);
	}
}

void turn_right(void){
	if (get_rescue()){
		left_motor_set_pos(ZERO);
		while (abs(left_motor_get_pos()) < 325){
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(-HIGH_SPEED);
		}
		while (get_calibrated_prox(IR6) > 150){
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		left_motor_set_pos(ZERO);
		while (abs(left_motor_get_pos()) < 400){
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		right_motor_set_pos(ZERO);
	}
}

void left_step(void){
	if (get_rescue()){
		left_motor_set_pos(ZERO);
		while (abs(left_motor_get_pos()) < 250){
			left_motor_set_speed(LOW_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		left_motor_set_pos(ZERO);
		while (abs(left_motor_get_pos()) < 500){
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(LOW_SPEED);
		}
		left_motor_set_pos(ZERO);
		left_motor_set_speed(HIGH_SPEED);
		right_motor_set_speed(HIGH_SPEED);
	}
}


void right_step(void){
	if (get_rescue()){
		right_motor_set_pos(ZERO);
		while (abs(right_motor_get_pos()) < 250){
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(LOW_SPEED);
		}
		right_motor_set_pos(ZERO);
		while (abs(right_motor_get_pos()) < 500){
			left_motor_set_speed(LOW_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		right_motor_set_pos(ZERO);
		left_motor_set_speed(HIGH_SPEED);
		right_motor_set_speed(HIGH_SPEED);
	}
}
