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
#include <pi_regulator.h>


//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);


//les valeur de la position du haut parleur par rapport aux micros
static int angle;
static double distance;
static int reference = 0;
static int nbr_passage = 0;
//static uint32_t puissance_source = 0;
//pour savoir approximativement d'ou vient le soin
static uint8_t cote_max;
//tableau pour stocker les valeurs des micros

static int moyenne[4][4]; // 4 pour nbr micro et 3 pour le nbr d'echantillon

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
static uint8_t coeff_capteur[8] = {0,0,0,0,0,0,0}; // en % -> /100



#define MIN_VALUE_THRESHOLD	10000

#define DISTANCE_REFERENCE 40000 // valeur équivalente à 6cm
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

/*#define PID_ERROR_THRESHOLD	0
#define ROTATION_THRESHOLD 	0
#define ROTATION_COEFF		1000
#define ANGLE_ERROR_COEFF	10
#define PID_KP 			800
#define PID_KI			3.5*/

#define NBR_ECHANTILLON 4
#define NBR_MICRO 4

#define MARGE_ANGLE 0.12 //%
#define VALEUR_MIN_PROXY 200



#define PERCENT


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

		int max_right = 0.98*get_max_norm_index(RIGHT_OUTPUT).max;
		int max_left = 0.98*get_max_norm_index(LEFT_OUTPUT).max;
		int max_front = get_max_norm_index(FRONT_OUTPUT).max;
		int max_back = 0.98*get_max_norm_index(BACK_OUTPUT).max; //facteur de correction pour faiblesse du micro
		int norm_right = get_max_norm_index(RIGHT_OUTPUT).norm;
		int norm_left = get_max_norm_index(LEFT_OUTPUT).norm;
		int norm_front = get_max_norm_index(FRONT_OUTPUT).norm;
		int norm_back = get_max_norm_index(BACK_OUTPUT).norm;

		//chprintf((BaseSequentialStream *) &SDU1,"max_right  = %d max_norm_index_right = %d\n,max_left = %d max_norm_index_left = %d \n",max_right, norm_right,max_left,norm_left);
		//chprintf((BaseSequentialStream *) &SDU1,"Max_front  = %d max_norm_index_front = %d\n,max_back = %d max_norm_index_back = %d \n\n\n",max_front,norm_front,max_back,norm_back);

		// rempli le tableau moyenne en fonction de la valeur de must send
		update_moyenne(mustSend, max_right,max_left,max_front,max_back);
		nbr_passage ++;
		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		//calcul la moyenne avec laquelle on va travailler
		if(mustSend > (NBR_ECHANTILLON-1)){

			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;



			int moyenne_right = calcul_moyenne(RIGHT);
			int moyenne_left = calcul_moyenne(LEFT);
			int moyenne_front = calcul_moyenne(FRONT);
			int moyenne_back = calcul_moyenne(BACK);

			led_direction(moyenne_right,moyenne_left,moyenne_front,moyenne_back);

			calcul_angle(moyenne_right,moyenne_left,moyenne_front,moyenne_back);
			cote_max = RIGHT;

			//calcul_distance(puissance_moyenne);
			//chprintf((BaseSequentialStream*)&SDU1,"moyenne_front = %d \n",moyenne_front);
			//chprintf((BaseSequentialStream*)&SDU1,"moyenne_right = %d \n, moyenne_left = %d \n, moyenne_front = %d \n,moyenne_back = %d \n\n\n\n ",moyenne_right,moyenne_left,moyenne_front,moyenne_back);
			//chprintf((BaseSequentialStream*)&SDU1,"angle = %d \n",angle);
			// APPEL DU PID POUR LOCALISER DOU VIENT LE SON ET TOURNER EN FONCTION

			//chprintf((BaseSequentialStream*)&SDU1,"moyenne_right  = %d \n " , moyenne_right);
			/*int speed_correction = pi_regulator(max_right, max_left, max_front, max_back);
			//chprintf((BaseSequentialStream *)&SD3, "max_front = %d \n", max_front);
			//if the sound is nearly in front of the camera, don't rotate
			if(abs(speed_correction) < ROTATION_THRESHOLD){ // SI ON EST DEJA EN FACE DU SON NE RIEN FAIRE
				speed_correction = 0;
			}

			//applies the speed from the PI regulator and the correction for the rotation
			right_motor_set_speed(ROTATION_COEFF * speed_correction);
			left_motor_set_speed(- ROTATION_COEFF * speed_correction);

		*/
		}

		nb_samples = 0;
		mustSend++;

		//sound_remote(micLeft_output);


	}
}

double calcul_distance(int32_t intensite_moyenne, int32_t puissance_source){

	double intensite_transition = intensite_moyenne;
	double distance_source = sqrt(((double)puissance_source)/(((double)intensite_transition)*4*3.1415) );

	return 2*distance_source;
}

void calcul_angle(int max_right,int max_left,int max_front,int max_back){

	uint8_t supremum = -1;

	int transition;
	int transition2;

	if(max_right>= max_left && max_right>=max_front && max_right>=max_back){
		transition = max_right*(1-MARGE_ANGLE);
		if(transition >= max_left && transition>=max_front && transition>=max_back){
			supremum = RIGHT;
		}else if(max_front>=transition){
			angle = 45;
			return;
		}else if(max_back >= transition){
			angle = 135;
			return;
		}
	}
	if (max_left>=max_front && max_left>= max_back && supremum!=-1){ // ?????? Tout le temps vrai ?
		transition = max_left*(1-MARGE_ANGLE);
		if(transition>=max_front && transition>= max_back){
			supremum = LEFT;
		}else if(max_front>=transition){
			angle = 315;
			return;
		}else if(max_back >= transition){
			angle = 225;
			return;
		}
	}

	if(max_front >=max_back && supremum != -1){
		transition = max_front*(1-MARGE_ANGLE);
		if (transition >= max_back)
			supremum = FRONT;
	}else if(supremum != -1){
		supremum = BACK ;
	}

	if(supremum ==FRONT){
		transition = max_left*(1-(MARGE_ANGLE));
		transition2 = max_right*(1-(MARGE_ANGLE));
		if(max_left >= max_right && transition >= max_right){
			angle = 337;
			return;
		}else if(max_left >=max_right && transition <= max_right){
			angle = 0;
			return;
		}else{
			angle = 22;
			//chprintf((BaseSequentialStream*)&SDU1,"transition  = %d \n, transition2 = %d, max_left = %d, max_right = %d,supremum = %d\n",transition,transition2,max_left,max_right,supremum);
			return;
		}

	}

	if(supremum ==RIGHT){
		transition = max_front*(1-MARGE_ANGLE);
		transition2 = max_back*(1-MARGE_ANGLE);
		if(max_front >= max_back && transition >= max_back){
			angle = 67;
			return;
		}else if(max_front >=max_back && transition <= max_back){
			angle = 90;
			return;
		}else{
			angle = 112;
			//chprintf((BaseSequentialStream*)&SDU1,"transition 1 = %d \n, transition2 = %d, max_front = %d, max_back = %d \n",transition,transition2,max_front,max_back);

			return;
		}

	}

	if(supremum ==BACK){
		transition = max_right*(1-MARGE_ANGLE);
		transition2 = max_left*(1-MARGE_ANGLE);
		if(max_right >= max_left && transition >= max_left){
			angle = 157;
			return;
		}else if(max_right >=max_left && transition <= max_left){
			angle = 180;
			return;
		}else{
			angle = 202;
			return;
		}
	}

	transition = max_back*(1-MARGE_ANGLE);
	transition2 = max_front*(1-MARGE_ANGLE);
	if(max_back >= max_front && transition >= max_front){
		angle = 247;
		return;
	}else if(max_back >=max_front && transition <= max_front){
		angle = 270;
		return;
	}else{
		angle = 292;
		return;
	}




return;

}


int calcul_moyenne(MICRO_NAME micro){
	int somme = 0;
	for(uint8_t i = 0; i < NBR_ECHANTILLON;i++){
		somme += moyenne[micro][i];
	}

	return (somme/3);
}


void update_moyenne(uint8_t mustSend,int max_right,int max_left,int max_front, int max_back){
	if (mustSend <4){
		moyenne[LEFT][mustSend-1] = max_left; //Left

		moyenne[RIGHT][mustSend-1] = max_right; //right
		moyenne[FRONT][mustSend-1] = max_front; //front
		moyenne[BACK][mustSend-1] = max_back; //back
		return;
	}else{
		return;
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
		//cote_max = RIGHT;
		return ;
	}else if(max_left>=max_front && max_left>= max_back){
		set_led(LED7,1);
		//cote_max = LEFT;
		return;
	}else if(max_front>=max_back){
		set_led(LED1,1);
		//cote_max = FRONT;
		return;
	}else{
		set_led(LED5,1);
		//cote_max = BACK ;
	}
	return;
	}

/*int pi_regulator(int max_right, int max_left, int max_front, int max_back){ // PID

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
}*/


void echantillone_distance(valeur){
	reference = (int)valeur;


	//chprintf((BaseSequentialStream *) &SDU1,"reference = %d\n nbr_passage = %d \n",reference,nbr_passage);
	return;


}

int get_max_front_moyenne(){
	return calcul_moyenne(FRONT);
}

int get_line_position(void){
	return angle;
}

float get_distance_cm(void){
	return distance;
}








DIRECTION_ROBOT calcul_direction(int vit_droit, int vit_gauche ){
	int borne_gauche_supp = 1.1*vit_gauche;
	int borne_gauche_min = 0.9*vit_gauche;
	if(vit_droit == 0 && vit_gauche == 0){
	return ARRET;
	//chprintf((BaseSequentialStream *) &SDU1,"ARRET\n");
	}

	if(vit_droit >=0 && vit_gauche>=0){
		if(vit_droit>vit_gauche && vit_droit>borne_gauche_supp){
			return DEVANT_GAUCHE;
		}else if((vit_droit>vit_gauche && borne_gauche_supp >vit_droit) || vit_gauche == vit_droit){
			return DEVANT;

		}else if(vit_gauche > vit_droit && borne_gauche_min<vit_droit ){
			return DEVANT;
		}else{
			return DEVANT_DROIT;
		}
	}else if(vit_droit<=0 && vit_gauche >=0){
		if((-vit_droit) == vit_gauche || ((-vit_droit) >vit_gauche && borne_gauche_supp > (-vit_droit))){
			return ARRET;
		}else if(vit_gauche>(-vit_droit) && (-vit_droit)>borne_gauche_min){
			return ARRET;
		}else{
			return DEVANT_DROIT;
		}

	}else if(vit_droit>=0 && vit_gauche<=0){
		if((-vit_gauche)== vit_droit || ((-vit_gauche)>vit_droit && (-borne_gauche_min)<vit_droit)){
			return ARRET;
		}else if(vit_droit>(-vit_gauche) && vit_droit <(-borne_gauche_supp) ){
			return ARRET;
		}else if(vit_droit >vit_gauche){
			return DEVANT_GAUCHE;
		}else{
			return DERRIERE_DROIT;
		}
	}else if(vit_droit<0 && vit_gauche<0){
		if(vit_droit>vit_gauche && vit_droit>borne_gauche_min){
			return DERRIERE_DROIT;
		}else if((vit_droit>vit_gauche && borne_gauche_min > vit_droit) || vit_gauche == vit_droit){
			return DERRIERE;
		}else if(vit_gauche > vit_droit && borne_gauche_supp < vit_droit ){
			return DERRIERE;
		}else{
			return DERRIERE_GAUCHE;
		}

	}else{
		return ARRET;
	}

}

void m_a_j_coeff_catpeurs(direction){
	if(direction == DEVANT){
		coeff_capteur[0] = 100;
		coeff_capteur[1] = 60;
		coeff_capteur[2] = 20;
		coeff_capteur[3] = 0;
		coeff_capteur[4] = 0;
		coeff_capteur[5] = 20;
		coeff_capteur[6] = 60;
		coeff_capteur[7] = 100;


	}else if(direction == DEVANT_DROIT){
		coeff_capteur[0] = 80;
		coeff_capteur[1] = 100;
		coeff_capteur[2] = 80;
		coeff_capteur[3] = 20;
		coeff_capteur[4] = 0;
		coeff_capteur[5] = 0;
		coeff_capteur[6] = 0;
		coeff_capteur[7] = 60;
	}else if(direction == DERRIERE_DROIT){
		coeff_capteur[0] = 0;
		coeff_capteur[1] = 30;
		coeff_capteur[2] = 80;
		coeff_capteur[3] = 100;
		coeff_capteur[4] = 80;
		coeff_capteur[5] = 20;
		coeff_capteur[6] = 0;
		coeff_capteur[7] = 0;

	}else if (direction == DERRIERE_GAUCHE){
		coeff_capteur[0] = 0;
		coeff_capteur[1] = 0;
		coeff_capteur[2] = 20;
		coeff_capteur[3] = 80;
		coeff_capteur[4] = 100;
		coeff_capteur[5] = 80;
		coeff_capteur[6] = 20;
		coeff_capteur[7] = 0;
	}else if (direction == DEVANT_GAUCHE){
		coeff_capteur[0] = 60;
		coeff_capteur[1] = 0;
		coeff_capteur[2] = 0;
		coeff_capteur[3] = 0;
		coeff_capteur[4] = 20;
		coeff_capteur[5] = 80;
		coeff_capteur[6] = 100;
		coeff_capteur[7] = 80;

	}else if (direction == DERRIERE){
		coeff_capteur[0] = 0;
		coeff_capteur[1] = 0;
		coeff_capteur[2] = 40;
		coeff_capteur[3] = 100;
		coeff_capteur[4] = 100;
		coeff_capteur[5] = 40;
		coeff_capteur[6] = 0;
		coeff_capteur[7] = 0;
	}else{
		coeff_capteur[0] = 0;
		coeff_capteur[1] = 0;
		coeff_capteur[2] = 0;
		coeff_capteur[3] = 0;
		coeff_capteur[4] = 0;
		coeff_capteur[5] = 0;
		coeff_capteur[6] = 0;
		coeff_capteur[7] = 0;
	}
}


int get_coeff_capteur(uint8_t i){
	return coeff_capteur[i];
}

