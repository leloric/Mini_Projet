#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>

#include <main.h>
#include <motor.h>
#include <audio_processing.h>
#include <obstacles.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>

#include <motors.h>
#include <leds.h>
#include <audio/microphone.h>
#include <msgbus/messagebus.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <audio/audio_thread.h>

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

//Thread pour la détection d'obstacles
static THD_WORKING_AREA(waThdObstacle, 128);
static THD_FUNCTION(ThdObstacle, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
        detection();
        chThdSleepMilliseconds(400);

    }
}


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    timer12_start();
    //initiate the motors
    motors_init();
	//initiate the bus
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	//starts proximity
	proximity_start();
	//starts the melody thread
	playMelodyStart();

    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it
    static float send_tab[FFT_SIZE];

    //starts the microphones processing thread.
    mic_start(&processAudioData);

    chThdSleepMilliseconds(2000);

    chThdCreateStatic(waThdObstacle, sizeof(waThdObstacle), NORMALPRIO+2, ThdObstacle, NULL);


    /* Infinite loop. */
    while (1) {

	if (get_rescue() && get_angle_found()){
		straight_track();
	}

	//we copy the buffer to avoid conflicts
	arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
	arm_copy_f32(get_audio_buffer_ptr(RIGHT_OUTPUT), send_tab, FFT_SIZE);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}


