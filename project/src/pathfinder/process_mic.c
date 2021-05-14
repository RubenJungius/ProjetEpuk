#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <leds.h>
#include <audio/microphone.h>
#include <process_mic.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micFront_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micFront_output[FFT_SIZE];
static int sequence_counter;
static int sequence_timer;
static messagebus_topic_t microphone_topic;
static microphone_msg_t mic_accepted;


#define MIN_VALUE_THRESHOLD	10000

//frequency=position*15,625 	  max pos = 512 (8kHz)
//current sequence is: 29/34/48
#define MIN_FREQ		20
#define FREQ_1			29	//453Hz
#define FREQ_2			34	//531Hz
#define FREQ_3			48	//750Hz
#define INTERVAL		1
#define SEQUENCE_TIME 	2000
#define MAX_FREQ		53//260	//we don't analyze after this index to not use resources for nothing

void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	mic_accepted.microphone_value = 0;
	//launch sequence
	if(sequence_counter == 0){
		if(max_norm_index >= FREQ_1 - INTERVAL && max_norm_index <= FREQ_1 + INTERVAL){
			sequence_counter++;
			sequence_timer=chVTGetSystemTime();
		}
	}else if(sequence_counter == 1){
		if(max_norm_index >= FREQ_2 - INTERVAL && max_norm_index <= FREQ_2 + INTERVAL)
			sequence_counter++;
	}else if(sequence_counter == 2){
		if(max_norm_index >= FREQ_3 - INTERVAL && max_norm_index <= FREQ_3 + INTERVAL)
			sequence_counter++;
	}
	if(sequence_counter != 0 && chVTTimeElapsedSinceX(sequence_timer) > SEQUENCE_TIME)
		sequence_counter = 0;

	if(sequence_counter == 3){ //sequence accepted
		sequence_counter = 0;
		mic_accepted.microphone_value = 1;
	}

	messagebus_topic_publish(&microphone_topic, &mic_accepted, sizeof(mic_accepted));
	//chprintf((BaseSequentialStream*)&SD3, "%d: %d, %f Hz %d\r\n",sequence_counter, max_norm_index, max_norm_index*15.625, chVTTimeElapsedSinceX(sequence_timer));
}

void init_counter(){
	sequence_counter = 0;
	sequence_timer = chVTGetSystemTime();
}
void init_messagebus(){
	MUTEX_DECL(mic_topic_lock);
	CONDVAR_DECL(mic_topic_condvar);
	messagebus_topic_init(&microphone_topic, &mic_topic_lock, &mic_topic_condvar, &mic_accepted, sizeof(mic_accepted));
	messagebus_advertise_topic(&bus, &microphone_topic, "/microphone");
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

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

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

		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);

		/*	Magnitude processing
		 *
		 *	Computes the magnitude of the complex numbers and
		 *	stores them in a buffer of FFT_SIZE because it only contains
		 *	real numbers.
		 *
		 */
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);

		nb_samples = 0;

		sound_remote(micFront_output);
	}
}
