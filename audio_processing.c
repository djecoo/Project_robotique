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
#include <leds.h>

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
#define MAX_FREQ		2*FFT_SIZE	//we don't analyze after this index to not use resources for nothing

#define FREQ_FORWARD_L		(FREQ_FORWARD-1)
#define FREQ_FORWARD_H		(FREQ_FORWARD+1)
#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_BACKWARD_L		(FREQ_BACKWARD-1)
#define FREQ_BACKWARD_H		(FREQ_BACKWARD+1)

#define PID_ERROR_THRESHOLD	0
#define ROTATION_THRESHOLD 	0
#define ROTATION_COEFF		1000
#define ANGLE_ERROR_COEFF	10
#define PID_KP 			800
#define PID_KI			3.5


/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/

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


	//go forward
	if(max_norm_index >= FREQ_FORWARD_L && max_norm_index <= FREQ_FORWARD_H){
		left_motor_set_speed(600);
		right_motor_set_speed(600);
	}
	//turn left
	else if(max_norm_index >= FREQ_LEFT_L && max_norm_index <= FREQ_LEFT_H){
		left_motor_set_speed(-600);
		right_motor_set_speed(600);
	}
	//turn right
	else if(max_norm_index >= FREQ_RIGHT_L && max_norm_index <= FREQ_RIGHT_H){
		left_motor_set_speed(600);
		right_motor_set_speed(-600);
	}
	//go backward
	else if(max_norm_index >= FREQ_BACKWARD_L && max_norm_index <= FREQ_BACKWARD_H){
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

			int max_right = get_max_norm_index(RIGHT_OUTPUT).max;
			int max_left = get_max_norm_index(LEFT_OUTPUT).max;
			int max_front = get_max_norm_index(FRONT_OUTPUT).max;
			int max_back = get_max_norm_index(BACK_OUTPUT).max;
			int norm_right = get_max_norm_index(RIGHT_OUTPUT).norm;
			int norm_left = get_max_norm_index(LEFT_OUTPUT).norm;
			int norm_front = get_max_norm_index(FRONT_OUTPUT).norm;
			int norm_back = get_max_norm_index(BACK_OUTPUT).norm;

			led_direction(max_right,max_left,max_front,max_back);
			//chprintf((BaseSequentialStream *) &SDU1,"max_right  = %d max_norm_index_right = %d\n,max_left = %d max_norm_index_left = %d \n",max_right, norm_right,max_left,norm_left);
			//chprintf((BaseSequentialStream *) &SDU1,"Max_front  = %d max_norm_index_front = %d\n,max_back = %d max_norm_index_back = %d \n\n\n",max_front,norm_front,max_back,norm_back);

			// APPEL DU PID POUR LOCALISER DOU VIENT LE SON ET TOURNER EN FONCTION

			int speed_correction = pi_regulator(max_right, max_left, max_front, max_back);
			//chprintf((BaseSequentialStream *)&SD3, "max_front = %d \n", max_front);
			//if the sound is nearly in front of the camera, don't rotate
			if(abs(speed_correction) < ROTATION_THRESHOLD){ // SI ON EST DEJA EN FACE DU SON NE RIEN FAIRE
				speed_correction = 0;
			}

			//applies the speed from the PI regulator and the correction for the rotation
			right_motor_set_speed(ROTATION_COEFF * speed_correction);
			left_motor_set_speed(- ROTATION_COEFF * speed_correction);


		}
		nb_samples = 0;
		mustSend++;

		sound_remote(micLeft_output);


	}
}

 maximum_fft max_norm(float* data){
	float max_norm = 100;
	int max_norm_index = -1;
	maximum_fft point = {0,0};


	//search for the highest peak and filters low frequencies
	for(int i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm && i>50 && i<1000){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	point.max = (int)max_norm;
	point.norm = max_norm_index;
	return point;
}


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

maximum_fft get_max_norm_index(BUFFER_NAME_t name){

	if (name == LEFT_OUTPUT){
		return max_norm(micLeft_output);
	}
	else if (name == RIGHT_OUTPUT){
		return max_norm(micRight_output);
	}
	else if (name == FRONT_OUTPUT){
		return max_norm(micFront_output);
	}
	else if (name == BACK_OUTPUT){
		return max_norm(micBack_output);
	}
	else{
		maximum_fft a = {0,0};
		return a;
	}
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


void led_direction(int max_right,int max_left,int max_front,int max_back){
	clear_leds();
	if(max_right>= max_left && max_right>=max_front && max_right>=max_back){
		set_led(LED3, 1);
		return ;
	}else if(max_left>=max_front && max_left>= max_back){
		set_led(LED7,1);
		return;
	}else if(max_front>=max_back){
		set_led(LED1,1);
		return;
	}else{
		set_led(LED5,1);
	}
	return;
	}

int pi_regulator(int max_right, int max_left, int max_front, int max_back){ // PID

	float error = 0; // DIFFERENCE
	float angle_correction = 0; // DIFFERENCE D'ANGLE ENTRE ROBOT / SON



	if (max_right > max_left && max_right > max_front && max_right > max_back) // TEST SI MICRO DROIT PLUS PROCHE
	{
		error = (max_front-max_back)/(max_front+max_left+max_back+max_right); // TEST DE QUEL COTE EST SON PAR RAPPORT AU MICRO DROIT
		angle_correction = -PI/2 + ANGLE_ERROR_COEFF* error; // CALCUL DE L'ANGLE PAR RAPPORT AU ROBOT, COEFF ANGLE_ERROR A CHANGER POUR OBTENIR VRAIE VALEUR D'ORDRE

		if(fabs(angle_correction) < PID_ERROR_THRESHOLD){ // SI ANGLE TROP FAIBLE RIEN FAIRE
				return 0;

		}
		chprintf((BaseSequentialStream *) &SDU1,"max_RIGHT  = %d max_front = %d\n,max_left = %d max_back = %d \n",max_right, max_front ,max_left,max_back);
		chprintf((BaseSequentialStream *) &SDU1,"PID_KP * angle_correction  = %d angle_correction = %d, error = %d\n",PID_KP * angle_correction, angle_correction, error);

		return (int16_t) PID_KP * angle_correction; // RETOUNER VALEUR POUR CHANGER VITESSE MOTEUR, KP A MODULER
	}

	if (max_back > max_left && max_back > max_front && max_back > max_right)
	{
			error = (max_right-max_left)/(max_front+max_left+max_back+max_right);
			if(error > 0){
				angle_correction = -PI + ANGLE_ERROR_COEFF* error;
			}

			if(error < 0){
				angle_correction = PI - ANGLE_ERROR_COEFF* error;
			}

			if(fabs(angle_correction) < PID_ERROR_THRESHOLD){
					return 0;

			}

			chprintf((BaseSequentialStream *) &SDU1,"max_right  = %d max_front = %d\n,max_left = %d max_BACK = %d \n",max_right, max_front ,max_left,max_back);
			chprintf((BaseSequentialStream *) &SDU1,"PID_KP * angle_correction  = %d angle_correction = %d, error = %d\n",PID_KP * angle_correction, angle_correction, error);

			return (int16_t) PID_KP * angle_correction;
	}

	if (max_left > max_right && max_left > max_front && max_left > max_back)
	{
			error = (max_front-max_back)/(max_front+max_left+max_back+max_right);
			angle_correction = PI/2 - ANGLE_ERROR_COEFF* error;

			if(fabs(angle_correction) < PID_ERROR_THRESHOLD){
					return 0;

			}

			chprintf((BaseSequentialStream *) &SDU1,"max_right  = %d max_front = %d\n,max_LEFT = %d max_back = %d \n",max_right, max_front ,max_left,max_back);
			chprintf((BaseSequentialStream *) &SDU1,"PID_KP * angle_correction  = %d angle_correction = %d, error = %d\n",PID_KP * angle_correction, angle_correction, error);

			return (int16_t) PID_KP * angle_correction;
	}

	if (max_front > max_right && max_front > max_left && max_front > max_back)
	{
			error = (max_left-max_right)/(max_front+max_left+max_back+max_right);
			angle_correction = 0 +ANGLE_ERROR_COEFF* error;

			if(fabs(angle_correction) < PID_ERROR_THRESHOLD){
						return 0;

			}

			chprintf((BaseSequentialStream *) &SDU1,"max_right  = %d max_FRONT = %d\n,max_left = %d max_back = %d \n",max_right, max_front ,max_left,max_back);
			chprintf((BaseSequentialStream *) &SDU1,"PID_KP * angle_correction  = %d angle_correction = %d, error = %d\n",PID_KP * angle_correction, angle_correction, error);

			return (int16_t) PID_KP * angle_correction;
	}

	return 0;
}




