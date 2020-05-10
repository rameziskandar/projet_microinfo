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
#include <audio/microphone.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <leds.h>

#include <audio_processing.h>
#include <TOF_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>



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

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


int main(void)
{
	messagebus_init(&bus, &bus_lock, &bus_condvar);
    halInit();
    chSysInit();
    mpu_init();
    //inits the LEDs
    clear_leds();
    set_body_led(0);
    set_front_led(0);
    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //inits the motors
    motors_init();
    //starts the mic communication
    mic_start(&processAudioData);
    //starts the TOF sensor communication
    VL53L0X_start();

    while (1)
    {
    	chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
