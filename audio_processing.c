#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <TOF_processing.h>
#include <fft.h>
#include <arm_math.h>
#include <sensors/VL53L0X/VL53L0X.h>

////semaphore
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

#define ROT_SPEED		500
#define AVANCE_SPEED	600

#define SOUND_THRESHOLD	14000	//minimum sound magnitude for detection
#define SOUND_STOP		60000	//sound magnitude when robot is underneath the source

#define FREQ_TARGET_1	16		//250Hz
#define FREQ_TARGET_2	23		//359Hz
#define FREQ_TARGET_3	29		//453HZ

#define INDEX_FREQ_1	12
#define INDEX_FREQ_2	24
#define INDEX_FREQ_3	36

#define PHASE_MIN		0.15 	//minimum threshold for phase difference values
#define PHASE_MAX		1.2		//maximum threshold for phase difference values
#define PHASE_STOP		0.11	//minimum threshold for phase difference values for stop

#define DIST_STOP		100  	//distance between robot and obstacle in mm



/*
*	Simple function used to detect the sound direction for a
*	given frequency and to move towards it.
*/
uint16_t sound_position_detection(uint8_t i, uint16_t freq){

	static float freq_norm_r;
	static float phase_right;
	static float phase_left;
	static float phase_back;
	static float phase_front;
	static float phase_diff_rl = 2;
	static float phase_diff_old_rl = 2;
	static float phase_diff_fb = 2;
	static float phase_diff_old_fb = 2;
	static uint16_t distance;

	distance = VL53L0X_get_dist_mm();

	//look for the magnitude of a given frequency
	freq_norm_r = micRight_output[freq];

	//compute the phase for each mic
	phase_right = atan2f(micRight_cmplx_input[2*freq+1],micRight_cmplx_input[2*freq]);
	phase_left  = atan2f(micLeft_cmplx_input[2*freq+1],micLeft_cmplx_input[2*freq]);
	phase_back  = atan2f(micBack_cmplx_input[2*freq+1],micBack_cmplx_input[2*freq]);
	phase_front = atan2f(micFront_cmplx_input[2*freq+1],micFront_cmplx_input[2*freq]);


	if(freq_norm_r > SOUND_THRESHOLD){
//		chprintf((BaseSequentialStream *)&SD3, "freq = %d\n", freq);
//		chprintf((BaseSequentialStream *)&SD3, "norm = %f\n", freq_norm_r);
		phase_diff_old_rl = phase_diff_rl;
		phase_diff_rl = phase_right-phase_left;

		//look if the phase difference between the right and left mic is almost equal to zero. We take 2 consecutive values
		//to be sure that we are not taking in consideration the errors from the mic
		if(phase_diff_rl < PHASE_MIN && phase_diff_rl > -PHASE_MIN &&
			phase_diff_old_rl < PHASE_MIN && phase_diff_old_rl > -PHASE_MIN){

			phase_diff_old_fb = phase_diff_fb;
			phase_diff_fb = phase_front - phase_back;

			//detecting if the robot is underneath the sound source
			if(phase_diff_fb < PHASE_STOP && phase_diff_fb > -PHASE_STOP &&
				phase_diff_old_fb < PHASE_STOP && phase_diff_old_fb > -PHASE_STOP &&
				phase_diff_rl < PHASE_STOP && phase_diff_rl > -PHASE_STOP &&
				phase_diff_old_rl < PHASE_STOP && phase_diff_old_rl > -PHASE_STOP &&
				freq_norm_r > SOUND_STOP){

				left_motor_set_speed(0);
				right_motor_set_speed(0);
				return i+1;
			}

			else if(phase_diff_fb < 0 && phase_diff_old_fb < 0 &&
					phase_diff_rl < PHASE_MIN && phase_diff_rl > -PHASE_MIN){

				left_motor_set_speed(-ROT_SPEED);
				right_motor_set_speed(ROT_SPEED);
				return i;
			}

			else if (distance < DIST_STOP){
				avoid_obstacle(distance);
				return i;
			}

			else{
					left_motor_set_speed(AVANCE_SPEED);
					right_motor_set_speed(AVANCE_SPEED);
					return i;
			}
		}

		//rotating the robot towards the sound source
		else if (phase_diff_rl > PHASE_MIN && phase_diff_rl < PHASE_MAX &&
			phase_diff_old_rl > PHASE_MIN && phase_diff_old_rl < PHASE_MAX ){

				left_motor_set_speed(ROT_SPEED);
				right_motor_set_speed(-ROT_SPEED);
				return i;

		}

		//rotating the robot towards the sound source
		else if (phase_diff_rl < -PHASE_MIN && phase_diff_rl > -PHASE_MAX &&
				 phase_diff_old_rl < -PHASE_MIN && phase_diff_old_rl > -PHASE_MAX){

				left_motor_set_speed(-ROT_SPEED);
				right_motor_set_speed(ROT_SPEED);
				return i;
		}
	}

	else{
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			return i;
	}

	return  i;
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
	static uint8_t index_freq = 0;

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

//		chprintf((BaseSequentialStream *)&SD3, "index = %d\n", index_freq);

		if (index_freq < INDEX_FREQ_1){
			index_freq = sound_position_detection(index_freq, FREQ_TARGET_1);
		}
		else if (index_freq < INDEX_FREQ_2){
			index_freq = sound_position_detection(index_freq, FREQ_TARGET_2);
		}
		else if (index_freq < INDEX_FREQ_3){
			index_freq = sound_position_detection(index_freq, FREQ_TARGET_3);
		}
		else if (index_freq >= INDEX_FREQ_3){
			index_freq=0;
		}
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
