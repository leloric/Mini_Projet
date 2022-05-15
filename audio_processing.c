#include "ch.h"
#include "hal.h"
#include <main.h>

#include <motors.h>
#include <selector.h>
#include <motor.h>
#include <audio/microphone.h>
#include <audio_processing.h>

#include <fft.h>
#include <arm_math.h>
#include <leds.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];

#define MIN_FREQ 		25
#define MAX_FREQ 		100
//we don’t analyze before and after this indexes to not use resources for nothing

//triangulation is better with small frequencies
#define FREQ_SOURCE_1_L			28 //110 Hz
#define FREQ_SOURCE_1_H			29 //125 Hz
#define FREQ_SOURCE_2_L 		17 //266 Hz
#define FREQ_SOURCE_2_H 		18 //281 Hz

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

#define MIN_VALUE_THRESHOLD	10000

#define ON					 1
#define OFF					 0
#define RESET_VALUE			 0
#define INDEX_0			    -1

#define MOITIE_SELECTOR		 8
#define ANGLE_MIN			 2
#define LOW_SPEED			200
#define MIDDLE_SPEED		300
#define HIGH_SPEED			500
#define INTENSITE_SUFFISANTE	170000


static _Bool rescue = OFF;
static _Bool angle_found = OFF;

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/

void sound_remote(float* data_left, float* data_right){
	float max_norm_left = MIN_VALUE_THRESHOLD;
	float max_norm_right = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index_left = -1;
	int16_t max_norm_index_right = -1;
	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data_left[i] > max_norm_left){
			max_norm_left = data_left[i];
			max_norm_index_left = i;
		}
		if(data_right[i] > max_norm_right){
			max_norm_right = data_right[i];
			max_norm_index_right = i;
		}
	}

	//enabling of the rescue
	if (get_selector() < MOITIE_SELECTOR){
		rescue = OFF;
		set_front_led(OFF);
		set_body_led(OFF);
		left_motor_set_speed(OFF);
		right_motor_set_speed(OFF);
		angle_found = OFF;
	} else {
		rescue = ON;
	}

	//source 1 a 450hz
	if(max_norm_index_left >= FREQ_SOURCE_1_L && max_norm_index_left <= FREQ_SOURCE_1_H && rescue && angle_found==OFF){
		int16_t angle = triangulation(max_norm_index_left, max_norm_index_right);
		if (max_norm_right>max_norm_left){
			rotate_clockwise(angle);
		} else {
			rotate_counter_clockwise(angle);
		}
		if (max_norm_right > INTENSITE_SUFFISANTE || max_norm_left > INTENSITE_SUFFISANTE){
			victim_found();
		}
	}

	//source 2 a 270hz
	if(max_norm_index_left >= FREQ_SOURCE_2_L && max_norm_index_left <= FREQ_SOURCE_2_H && rescue && angle_found==OFF){
		int16_t angle = triangulation(max_norm_index_left, max_norm_index_right);
		if (max_norm_right>max_norm_left){
			rotate_clockwise(angle);
		} else {
			rotate_counter_clockwise(angle);
		}
		if (max_norm_right > INTENSITE_SUFFISANTE || max_norm_left > INTENSITE_SUFFISANTE){
			victim_found();
		}
	}
}

int16_t triangulation(int16_t frequ_left, int16_t frequ_right){
	float phase_right = atan2f(micRight_cmplx_input[2*frequ_right+1], micRight_cmplx_input[2*frequ_right]);
	float phase_left = atan2f(micLeft_cmplx_input[2*frequ_left+1], micLeft_cmplx_input[2*frequ_left]);
	int16_t dephasage=(phase_right-phase_left)*180/PI;
	return dephasage;
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/

void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/


	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;

		//sound_remote(micLeft_output);
		sound_remote(micLeft_output, micRight_output);
	}
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}

_Bool get_rescue(void){
	return rescue;
}

_Bool get_angle_found(void){
	return angle_found;
}

void set_angle_found(_Bool state) {
	angle_found = state;
}
