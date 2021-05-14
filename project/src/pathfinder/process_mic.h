#ifndef PROCESS_MIC_H_
#define PROCESS_MIC_H_


#define FFT_SIZE 	1024

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

typedef struct{
	int microphone_value;
}microphone_msg_t;

void init_counter(void);
void processAudioData(int16_t *data, uint16_t num_samples);
void init_messagebus(void);

#endif /* PROCESS_MIC_H_ */
