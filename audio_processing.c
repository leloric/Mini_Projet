#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <leds.h>
#include <selector.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include <math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

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
#define MAX_NORM_INDEX  -1

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing
#define FREQ_FORWARD	16	//250Hz
#define FREQ_LEFT		19	//296Hz
#define FREQ_RIGHT		23	//359HZ
#define FREQ_BACKWARD	26	//406Hz
#define FREQ_FORWARD_L		(FREQ_FORWARD-1)
#define FREQ_FORWARD_H		(FREQ_FORWARD+1)
#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_BACKWARD_L		(FREQ_BACKWARD-1)
#define FREQ_BACKWARD_H		(FREQ_BACKWARD+1)

#define NSTEP_ONE_TURN 1283 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER 13 // [cm]



/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/

void sound_remote(float* data_left, float* data_right, float* data_front, float* data_back){

	float max_norm_left = MIN_VALUE_THRESHOLD;
	float max_norm_right = MIN_VALUE_THRESHOLD;
	float max_norm_front = MIN_VALUE_THRESHOLD;
	float max_norm_back = MIN_VALUE_THRESHOLD;

	int16_t max_norm_index_left = MAX_NORM_INDEX;
	int16_t max_norm_index_right = MAX_NORM_INDEX;
	//int16_t max_norm_index_front = MAX_NORM_INDEX;
	//int16_t max_norm_index_back = MAX_NORM_INDEX;

	//search for the highest peak of each mic
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
			if(data_left[i] > max_norm_left){
				max_norm_left = data_left[i];
				max_norm_index_left = i;
			}
			if(data_right[i] > max_norm_right){
				max_norm_right = data_right[i];
				max_norm_index_right = i;
			}
			if(data_front[i] > max_norm_front){
				max_norm_front = data_front[i];
				//max_norm_index_front = i;
			}
			if(data_back[i] > max_norm_back){
				max_norm_back = data_back[i];
				//max_norm_index_back = i;
			}
		}

	//go forward
	if(max_norm_index_left >= FREQ_FORWARD_L && max_norm_index_left <= FREQ_FORWARD_H){
		find_angle(max_norm_index_left, max_norm_index_right, max_norm_front, max_norm_back);	// 250 Hz
	}
	//go backward
		/*else{
		left_motor_set_speed(-600);
		right_motor_set_speed(-600);
	}*/
	
}

void rotation (float angle){
	clear_leds();
	set_front_led(1);
	angle = 3*PI/4;
	int32_t nb_steps = angle*NSTEP_ONE_TURN/(2*PI);
	//int32_t pos_init_left = left_motor_get_pos();
	int32_t pos_left = left_motor_get_pos();
	if (angle > 0){
		left_motor_set_speed(-200);
		right_motor_set_speed(200);
	}
	else {
		left_motor_set_speed(200);
		right_motor_set_speed(-200);
	}
	do{
		pos_left = abs(left_motor_get_pos());
	}while (pos_left < abs(nb_steps));
	set_body_led(1);
	int stop = 0;
	do{
		left_motor_set_speed(300);
		right_motor_set_speed(300);
	} while (stop == 0);
}

void find_angle(int16_t max_norm_index_left, int16_t max_norm_index_right, float max_norm_front, float max_norm_back){
	float phase_left = atan2f(micLeft_cmplx_input[2*max_norm_index_left+1], micLeft_cmplx_input[2*max_norm_index_left]); //[rad]
	float phase_right = atan2f(micRight_cmplx_input[2*max_norm_index_right+1], micRight_cmplx_input[2*max_norm_index_right]); //[rad]
	float dephasage= phase_left - phase_right; //compute the phase difference, which gives the angle	[rad]

	clear_leds();

	if (max_norm_back > max_norm_front){
		if (dephasage < 0){dephasage = dephasage - PI/2;}
		else {dephasage = dephasage + PI/2;}
	}

	if(dephasage >= PI/2 && dephasage < PI){set_led(LED7,1);	set_led(LED5,1);}
	if(dephasage >= 0 && dephasage < PI/2){set_led(LED7,1);	set_led(LED1,1);}
	if(dephasage >= -PI/2 && dephasage < 0){set_led(LED1,1);		set_led(LED3,1);}
	if(dephasage >= -PI && dephasage < -PI/2){set_led(LED3,1);	set_led(LED5,1);}

	rotation(dephasage);
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
//void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	/*static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

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
*/

void processAudioData(int16_t *data, uint16_t num_samples){
	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;
	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;

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
		*/

		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);


		//sends to UART3
		if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}

		//sends only one FFT result over 10 for 1 mic to not flood the computer

		nb_samples = 0;
		mustSend++;
		sound_remote(micLeft_output, micRight_output, micFront_output, micBack_output);
	}
}

/*void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}*/

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
