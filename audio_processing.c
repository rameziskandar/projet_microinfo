#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include <sensors/VL53L0X/VL53L0X.h>

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

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define FREQ_FORWARD	16	//250Hz
#define FREQ_LEFT		19	//296Hz
#define FREQ_RIGHT		23	//359HZ
#define FREQ_BACKWARD	26	//406Hz
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing
#define PHASE_MIN		0.15 //minimum threshold for phase difference values
#define PHASE_MAX		2 	//maximum threshold for phase difference values
#define DIST_STOP   70  //distance between robot and obstacle in mm


#define FREQ_FORWARD_L		(FREQ_FORWARD-1)
#define FREQ_FORWARD_H		(FREQ_FORWARD+1)
#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_BACKWARD_L		(FREQ_BACKWARD-1)
#define FREQ_BACKWARD_H		(FREQ_BACKWARD+1)



int16_t peak_frequency(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t freq_index = -1;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			freq_index = i;
		}
	}
	return freq_index;
}

/*
*	Simple function used to detect the highest value in a buffer
*	and to find where the sound comes from.
*/
void sound_position_detection(void){

	static int16_t freq_index_r;
	static float phase_right;
	static float phase_left;
	static float phase_back;
	static float phase_diff = 2;
	static float phase_diff_old = 2;
	static uint16_t distance;

	distance = VL53L0X_get_dist_mm();

	freq_index_r = peak_frequency(micRight_output);
	phase_right = atan2f(micRight_cmplx_input[2*freq_index_r+1],micRight_cmplx_input[2*freq_index_r]);

	phase_left = phase_mic(freq_index_r, micLeft_output, micLeft_cmplx_input);
	phase_back = phase_mic(freq_index_r, micBack_output, micBack_cmplx_input);

	if(freq_index_r >= FREQ_FORWARD_L && freq_index_r <= FREQ_FORWARD_H){
		phase_diff_old = phase_diff;
		phase_diff = phase_right-phase_left;

		if (phase_diff > PHASE_MIN && phase_diff < PHASE_MAX &&
			phase_diff_old > PHASE_MIN && phase_diff_old < PHASE_MAX ){

				left_motor_set_speed(600);
				right_motor_set_speed(-600);

		}
		else if (phase_diff < -PHASE_MIN && phase_diff > -PHASE_MAX &&
				 phase_diff_old < -PHASE_MIN && phase_diff_old > -PHASE_MAX){

				left_motor_set_speed(-600);
				right_motor_set_speed(600);

		}
		else if(phase_diff < PHASE_MIN && phase_diff > -PHASE_MIN &&
				phase_diff_old < PHASE_MIN && phase_diff_old > -PHASE_MIN){

			if ((phase_back-phase_left) > 0 && (phase_back-phase_left) <= (phase_back-phase_right) + 0.1 &&
				(phase_back-phase_left) >= (phase_back-phase_right) - 0.1){

				left_motor_set_speed(-600);
				right_motor_set_speed(600);
			}
			else if (distance < DIST_STOP){
				left_motor_set_speed(0);
				right_motor_set_speed(0);
			}

			else{
					left_motor_set_speed(600);
					right_motor_set_speed(600);
			}
		}
	}
	else{
			left_motor_set_speed(0);
			right_motor_set_speed(0);
	}
//	chprintf((BaseSequentialStream *)&SD3, "phase1 = %f\n", phase_right - phase_left);
	chprintf((BaseSequentialStream *)&SD3, "phase2 = %f\n", phase_diff);
//	chprintf((BaseSequentialStream *)&SD3, "phase3 = %f\n", phase_back);
}

float phase_mic (int16_t reference, float* data1, float* data2){

	static int16_t freq_index;
	static float phase;

	freq_index = peak_frequency(data1);
	if(freq_index <= reference + 1 && freq_index >= reference - 1 ){
		phase = atan2f(data2[2*freq_index+1],data2[2*freq_index]);
		return phase;
	}
	else {
		return 0;
	}
}

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* data){

	static uint16_t distance;
	static int16_t freq_index;

	freq_index = peak_frequency(data);
	distance = VL53L0X_get_dist_mm();

	//go forward
	if(freq_index >= FREQ_FORWARD_L && freq_index <= FREQ_FORWARD_H){
		if (distance < 100) {
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
		else {
			left_motor_set_speed(600);
			right_motor_set_speed(600);
		}
	}
	//turn left
	else if(freq_index >= FREQ_LEFT_L && freq_index <= FREQ_LEFT_H){
		left_motor_set_speed(-600);
		right_motor_set_speed(600);
	}
	//turn right
	else if(freq_index >= FREQ_RIGHT_L && freq_index <= FREQ_RIGHT_H){
		left_motor_set_speed(600);
		right_motor_set_speed(-600);
	}
	//go backward
	else if(freq_index >= FREQ_BACKWARD_L && freq_index <= FREQ_BACKWARD_H){
		left_motor_set_speed(-600);
		right_motor_set_speed(-600);
	}
	else{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
	
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

//		sound_remote(micLeft_output);
		sound_position_detection();
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
