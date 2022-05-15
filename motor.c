#include <stdlib.h>
#include <stdint.h>

#include <motor.h>
#include <audio_processing.h>

#include <motors.h>
#include <leds.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>


#define HIGH_SPEED			500
#define LOW_SPEED			200
#define HALF_TURN 			650
#define ANGLE_MIN			3
#define PETITE_ATTENTE		300
#define SMALL_TURN			250
#define BIG_TURN 			500
#define BIG_MARGIN          375
#define SMALL_MARGIN        75
#define STEP 				400
#define BOUCLE 				7
#define IR3					2
#define IR6					5
#define ON					1
#define OFF					0



void straight_track(void){
	if (get_rescue()){
		left_motor_set_speed(HIGH_SPEED);
		right_motor_set_speed(HIGH_SPEED);
	}
}

//effectue une rotatio perpetuelle
void turn (void){
	if (get_rescue()){
		set_body_led(ON);
		left_motor_set_speed(-HIGH_SPEED);
		right_motor_set_speed(HIGH_SPEED);
	}
}

//effectue un demi-tour
void turn_back (void){
	if (get_rescue()){
		right_motor_set_pos(OFF);
		while (abs(right_motor_get_pos())<HALF_TURN){
			left_motor_set_speed(-HIGH_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		left_motor_set_speed(OFF);
		right_motor_set_speed(OFF);
	}
	right_motor_set_pos(OFF);
}

//celebration quand la victime a été trouvée
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
		left_motor_set_speed(OFF);
		right_motor_set_speed(OFF);
	}
}

void rotate_clockwise(int16_t angle){
	if (get_rescue()) {
		if (abs(angle) > ANGLE_MIN && get_angle_found()==OFF){
			set_body_led(ON);
			set_front_led(OFF);
			left_motor_set_speed(LOW_SPEED);
			right_motor_set_speed(-LOW_SPEED);
		} else {
			set_front_led(ON);
			set_body_led(OFF);
			set_angle_found(ON);
			left_motor_set_speed(OFF);
			right_motor_set_speed(OFF);
		}
	}
}

void rotate_counter_clockwise(int16_t angle){
	if (get_rescue()) {
		if (abs(angle) > ANGLE_MIN && get_angle_found()==OFF){
			set_body_led(ON);
			set_front_led(OFF);
			left_motor_set_speed(-LOW_SPEED);
			right_motor_set_speed(LOW_SPEED);
		} else {
			set_front_led(ON);
			set_body_led(OFF);
			set_angle_found(ON);
			left_motor_set_speed(OFF);
			right_motor_set_speed(OFF);
		}
	}
}

//tourne 90 degres a gauche
void turn_left(void){
	if (get_rescue()){
		right_motor_set_pos(OFF);
		while (abs(right_motor_get_pos()) < BIG_MARGIN){
			left_motor_set_speed(-HIGH_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		while (get_calibrated_prox(IR3) > SMALL_MARGIN){
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		right_motor_set_pos(OFF);
		while (abs(right_motor_get_pos()) < STEP){
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		right_motor_set_pos(OFF);
	}
}

//tourne 90 degres a droite
void turn_right(void){
	if (get_rescue()){
		left_motor_set_pos(OFF);
		while (abs(left_motor_get_pos()) < BIG_MARGIN){
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(-HIGH_SPEED);
		}
		while (get_calibrated_prox(IR6) > SMALL_MARGIN){
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		left_motor_set_pos(OFF);
		while (abs(left_motor_get_pos()) < STEP){
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		right_motor_set_pos(OFF);
	}
}

//léger décalage a gauche
void left_step(void){
	if (get_rescue()){
		left_motor_set_pos(OFF);
		while (abs(left_motor_get_pos()) < SMALL_TURN){
			left_motor_set_speed(LOW_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		left_motor_set_pos(OFF);
		while (abs(left_motor_get_pos()) < BIG_TURN){
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(LOW_SPEED);
		}
		left_motor_set_pos(OFF);
		left_motor_set_speed(HIGH_SPEED);
		right_motor_set_speed(HIGH_SPEED);
	}
}

//léger décalage a droite
void right_step(void){
	if (get_rescue()){
		right_motor_set_pos(OFF);
		while (abs(right_motor_get_pos()) < SMALL_TURN){
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(LOW_SPEED);
		}
		right_motor_set_pos(OFF);
		while (abs(right_motor_get_pos()) < BIG_TURN){
			left_motor_set_speed(LOW_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		right_motor_set_pos(OFF);
		left_motor_set_speed(HIGH_SPEED);
		right_motor_set_speed(HIGH_SPEED);
	}
}
