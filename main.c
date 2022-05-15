#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <motor.h>
#include <audio/microphone.h>
#include <msgbus/messagebus.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <audio/microphone.h>
#include <audio/audio_thread.h>

#include <sensors/VL53L0X/VL53L0X.h>
#include "obstacles.h"
#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
#include "leds.h"


//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING
#define IR1						0
#define IR3						2
#define IR6						5
#define IR8						7
#define ATTENTE		7000
#define ON		1
#define OFF		0
#define LOW_SPEED	200
#define HIGH_SPEED	500
#define STEP		500
#define ZERO		0

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);



static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


void left_step(void){

		left_motor_set_pos(ZERO);
		while (abs(left_motor_get_pos()) < STEP){
			left_motor_set_speed(LOW_SPEED);
			right_motor_set_speed(HIGH_SPEED);
		}
		left_motor_set_pos(ZERO);
		while (abs(left_motor_get_pos()) < STEP){
			left_motor_set_speed(HIGH_SPEED);
			right_motor_set_speed(LOW_SPEED);
		}
		left_motor_set_pos(ZERO);
		left_motor_set_speed(HIGH_SPEED);
		right_motor_set_speed(HIGH_SPEED);
	}

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
		while(j<7) {
			set_led(LED1, 1);
			chThdSleepMilliseconds(300);
			set_led(LED3, 1);
			chThdSleepMilliseconds(300);
			set_led(LED5, 1);
			chThdSleepMilliseconds(300);
			set_led(LED7, 1);
			chThdSleepMilliseconds(300);
			set_led(LED1, 0);
			chThdSleepMilliseconds(300);
			set_led(LED3, 0);
			chThdSleepMilliseconds(300);
			set_led(LED5, 0);
			chThdSleepMilliseconds(300);
			set_led(LED7, 0);
			++j;
		}
		set_body_led(0);
		set_front_led(1);
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
}

void right_step(void){

		right_motor_set_pos(0);
		while (abs(right_motor_get_pos())<200){
			left_motor_set_speed(500);
			right_motor_set_speed(200);
		}
		right_motor_set_pos(0);
		while (abs(right_motor_get_pos())<500){
			left_motor_set_speed(200);
			right_motor_set_speed(500);
		}
		right_motor_set_pos(ZERO);
		left_motor_set_speed(500);
		right_motor_set_speed(500);
	}


void straight_track(void){
	left_motor_set_speed(500);
	right_motor_set_speed(500);
}

static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    timer12_start();
    //inits the motors
    motors_init();



    //initiate the bus

    //starts proximity
	proximity_start();
	//digital analogic converter
	dac_start();

	playMelodyStart();
	//get_ambient_light();
	//calibrate_ir();






    //temp tab used to store values in complex_float format
    //needed bx doFFT_c
   // static complex_float temp_tab[FFT_SIZE];

    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it
    static float send_tab[FFT_SIZE];


#ifdef SEND_FROM_MIC
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);

#endif  /* SEND_FROM_MIC */


    /* Infinite loop. */
    while (1) {
	if (get_rescue()){
		//set_front_led(ON);
		chThdSleepMilliseconds(ATTENTE);
	}



#ifdef SEND_FROM_MIC
        //waits until a result must be sent to the computer
        wait_send_to_computer();
#ifdef DOUBLE_BUFFERING
        //we copy the buffer to avoid conflicts
        arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
        SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, FFT_SIZE);
#else
        SendFloatToComputer((BaseSequentialStream *) &SD3, get_audio_buffer_ptr(LEFT_OUTPUT), FFT_SIZE);
#endif  /* DOUBLE_BUFFERING */
#else
        //time measurement variables
        volatile uint16_t time_fft = 0;
        volatile uint16_t time_mag  = 0;

        float* bufferCmplxInput = get_audio_buffer_ptr(LEFT_CMPLX_INPUT);
        float* bufferOutput = get_audio_buffer_ptr(LEFT_OUTPUT);

        uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, bufferCmplxInput, FFT_SIZE);

        if(size == FFT_SIZE){
            /*
            *   Optimized FFT
            */
            
            chSysLock();
            //reset the timer counter
            GPTD12.tim->CNT = 0;

            doFFT_optimized(FFT_SIZE, bufferCmplxInput);

            time_fft = GPTD12.tim->CNT;
            chSysUnlock();


            chSysLock();
            //reset the timer counter
            GPTD12.tim->CNT = 0;

            arm_cmplx_mag_f32(bufferCmplxInput, bufferOutput, FFT_SIZE);

            time_mag = GPTD12.tim->CNT;
            chSysUnlock();

            SendFloatToComputer((BaseSequentialStream *) &SD3, bufferOutput, FFT_SIZE);
            //chprintf((BaseSequentialStream *) &SDU1, "time fft = %d us, time magnitude = %d us\n",time_fft, time_mag);

        }
#endif  /* SEND_FROM_MIC */
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}


