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


//les valeur de la position du haut parleur par rapport aux micros
static int angle;

//tableau pour stocker les valeurs des micros
// 4 pour nbr micro et 3 pour le nbr d'echantillon

static int moyenne[4][4];
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
#define FREQ_MODE1	66	//1000Hz
#define FREQ_MODE2		79	//1200
#define FREQ_MODE3		92//1400Hz
#define MAX_FREQ		2*FFT_SIZE	//we don't analyze after this index to not use resources for nothing



#define FREQ_MODE1_L		(FREQ_MODE1-1)
#define FREQ_MODE1_H		(FREQ_MODE1+1)
#define FREQ_MODE2_L			(FREQ_MODE2-1)
#define FREQ_MODE2_H			(FREQ_MODE2+1)
#define FREQ_MODE3_L		(FREQ_MODE3-1)
#define FREQ_MODE3_H		(FREQ_MODE3+1)

//used to store the old value in case if the new one are not usabl
static int old_max_right;
static int old_max_left;
static int old_max_front;
static int old_max_back;
static uint8_t mode = 0;


#define NBR_ECHANTILLON 4
#define NBR_MICRO 4

#define MARGE_ANGLE 0.12 // %
#define VALEUR_MIN_PROXY 100



#define PERCENT

//function interne au fichier a definir car utilisée avant leur definition

void check_mode(int max_right,int max_left,int max_front,int max_back,int norm_right,int norm_left,int norm_front,int norm_back,uint8_t mustSend);

void led_direction(int max_right,int max_left,int max_front,int max_back);

void update_moyenne(uint8_t mustSend,int max_right,int max_left,int max_front, int max_back);


/*
*	this functiun read the value of the maximum and determine if it is
*	from the sound. We have a lot of noise with the mic so try
*	so id we have 2 value int the range we configure the mode
*	we use old value to fill the empty one
*/


void check_mode(int max_right,int max_left,int max_front,int max_back,int norm_right,int norm_left,int norm_front,int norm_back,uint8_t mustSend){

		uint8_t mode1_compteur = 0;
		uint8_t mode2_compteur = 0;
		uint8_t mode3_compteur = 0;

		//the function now check for every mic which one correspond to which mode
		if(norm_left >= FREQ_MODE1_L && norm_left <= FREQ_MODE1_H){
			mode1_compteur ++;
		}
		else if(norm_left >= FREQ_MODE2_L && norm_left <= FREQ_MODE2_H){
			mode2_compteur++;
		}
		else if(norm_left >= FREQ_MODE3_L && norm_left <= FREQ_MODE3_H){
			mode3_compteur++;
		}

		if(norm_right >= FREQ_MODE1_L && norm_right <= FREQ_MODE1_H){
			mode1_compteur++;
		}
		else if(norm_right >= FREQ_MODE2_L && norm_right <= FREQ_MODE2_H){
			mode2_compteur++;
		}
		else if(norm_right >= FREQ_MODE3_L && norm_right <= FREQ_MODE3_H){
			mode3_compteur++;
		}

		if(norm_front >= FREQ_MODE1_L && norm_front <= FREQ_MODE1_H){
			mode1_compteur ++;
		}
		else if(norm_front >= FREQ_MODE2_L && norm_front <= FREQ_MODE2_H){
			mode2_compteur++;
		}
		else if(norm_front >= FREQ_MODE3_L && norm_front <= FREQ_MODE3_H){
			mode3_compteur ++;
		}

		if(norm_back >= FREQ_MODE1_L && norm_back <= FREQ_MODE1_H){
			mode1_compteur ++;
		}
		else if(norm_back >= FREQ_MODE2_L && norm_back <= FREQ_MODE2_H){
			mode2_compteur++;
		}
		else if(norm_back >= FREQ_MODE3_L && norm_back <= FREQ_MODE3_H){
			mode3_compteur++;
		}


		//The function now compares every result for each mode and
		//select which data he can uses
		if(mode1_compteur >=2){
			mode = 1;
			if(norm_left < FREQ_MODE1_L || norm_left > FREQ_MODE1_H)
				max_left  = old_max_left;
			else
				old_max_left = max_left;
			if (norm_right < FREQ_MODE1_L || norm_right > FREQ_MODE1_H)
				max_right = old_max_right;
			else
				old_max_right = max_right;
			if(norm_front < FREQ_MODE1_L || norm_front > FREQ_MODE1_H)
				max_front = old_max_front;
			else
				old_max_front = max_front;
			if (norm_back < FREQ_MODE1_L || norm_back > FREQ_MODE1_H)
				max_back = old_max_back;
			else
				old_max_back = max_back;

		}else if(mode2_compteur >=2){
			mode = 2;
			if(norm_left < FREQ_MODE2_L || norm_left > FREQ_MODE2_H)
				max_left  = old_max_left;
			else
				old_max_left = max_left;
			if (norm_right < FREQ_MODE2_L || norm_right > FREQ_MODE2_H)
				max_right = old_max_right;
			else
				old_max_right = max_right;
			if(norm_front < FREQ_MODE2_L || norm_front > FREQ_MODE2_H)
				max_front = old_max_front;
			else
				old_max_front = max_front;
			if (norm_back < FREQ_MODE2_L || norm_back > FREQ_MODE2_H)
				max_back = old_max_back;
			else
				old_max_back = max_back;

		}else if(mode3_compteur>=2){
			mode = 3;
			if(norm_left < FREQ_MODE3_L || norm_left > FREQ_MODE3_H)
				max_left  = old_max_left;
			else
				old_max_left = max_left;
			if (norm_right < FREQ_MODE3_L || norm_right > FREQ_MODE3_H)
				max_right = old_max_right;
			else
				old_max_right = max_right;
			if(norm_front < FREQ_MODE3_L || norm_front > FREQ_MODE3_H)
				max_front = old_max_front;
			else
				old_max_front = max_front;
			if (norm_back < FREQ_MODE3_L || norm_back > FREQ_MODE3_H)
				max_back = old_max_back;
			else
				old_max_back = max_back;

		}else{
			mode = 0;
		}

	update_moyenne(mustSend, max_right,max_left,max_front,max_back);

	return ;
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

		//get the max norme for each mic and the norm associated with
		int max_right = 0.96*get_max_norm_index(RIGHT_OUTPUT).max;
		int max_left = 0.98*get_max_norm_index(LEFT_OUTPUT).max;
		int max_front = get_max_norm_index(FRONT_OUTPUT).max;
		int max_back = 0.98*get_max_norm_index(BACK_OUTPUT).max; //facteur de correction pour faiblesse du micro

		int norm_right = get_max_norm_index(RIGHT_OUTPUT).norm;
		int norm_left = get_max_norm_index(LEFT_OUTPUT).norm;
		int norm_front = get_max_norm_index(FRONT_OUTPUT).norm;
		int norm_back = get_max_norm_index(BACK_OUTPUT).norm;

		//va décoder les valeurs et remplit le tableau de moyenne et mode en fonction
		check_mode(max_right,max_left,max_front,max_back,norm_right,norm_left,norm_front,norm_back,mustSend);

		//only work with the average of NBR_ECHANTILLON
		if(mustSend > (NBR_ECHANTILLON-1)){

			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;


			int moyenne_right = calcul_moyenne(RIGHT);
			int moyenne_left = calcul_moyenne(LEFT);
			int moyenne_front = calcul_moyenne(FRONT);
			int moyenne_back = calcul_moyenne(BACK);

			//allume les leds
			led_direction(moyenne_right,moyenne_left,moyenne_front,moyenne_back);
			//calcule l'angle
			calcul_angle(moyenne_right,moyenne_left,moyenne_front,moyenne_back);
		}

		nb_samples = 0;
		mustSend++;
	}
}

uint8_t get_mode(void){
	return mode;
}

/*
 * Permet de sortir la distance en cm en fonction de la puissance originale et de la nouvelle puissance
 */
double calcul_distance(int32_t intensite_moyenne, int32_t puissance_source){


	double intensite_transition = intensite_moyenne;
	//formule determinee empiriquement
	double distance_source = sqrt(((double)puissance_source)/(((double)intensite_transition)*4*3.1415) );

	return 2*distance_source;
}

/*
 * Cette fonction calcule l'angle de la source en fonction des valeurs des micros
 * Elle permet d'avoir une resolution de 22.5deg (sans le bruit)
 *
 */
void calcul_angle(int max_right,int max_left,int max_front,int max_back){

	int8_t supremum = -1;

	int transition;

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
		if(max_left >= max_right && transition >= max_right){
			angle = 337;
			return;
		}else if(max_left >=max_right && transition <= max_right){
			angle = 0;
			return;
		}else{
			angle = 22;
			return;
		}

	}

	if(supremum ==RIGHT){
		transition = max_front*(1-MARGE_ANGLE);
		if(max_front >= max_back && transition >= max_back){
			angle = 67;
			return;
		}else if(max_front >=max_back && transition <= max_back){
			angle = 90;
			return;
		}else{
			angle = 112;
			return;
		}

	}

	if(supremum ==BACK){
		transition = max_right*(1-MARGE_ANGLE);
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


/*
 * Permet de calculer la moyenne des dernières valeurs vues par les micros
 */
int calcul_moyenne(MICRO_NAME micro){

	int somme = 0;
	for(uint8_t i = 0; i < NBR_ECHANTILLON;i++){
		somme += moyenne[micro][i];
	}

	return (somme/NBR_ECHANTILLON);
}


//permet de remplis le tableau moyenne en fonction de où on en est
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

/*
 * Cette fonction permet de renvoyer sous forme de struct le maximum de data ainsi que la norme associée
 */
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


//Cette fonction permet d'allumer les leds en fonction de quelle micro détecte la plus grande
//intensité
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


int get_max_front_moyenne(){
	return calcul_moyenne(FRONT);
}

int get_line_position(void){
	return angle;
}



/*
 * Cette fonction permet de voir vers ou va le robot en fonction des ordes donnés aux moteurs
 * Elle est utilisées pour permettre de mettre des poids aux capteurs de proximité
 */
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

void m_a_j_coeff_catpeurs(DIRECTION_ROBOT direction){
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
	}else if(direction == ARRET){
		coeff_capteur[0] = 100;
		coeff_capteur[1] = 100;
		coeff_capteur[2] = 100;
		coeff_capteur[3] = 100;
		coeff_capteur[4] = 100;
		coeff_capteur[5] = 100;
		coeff_capteur[6] = 100;
		coeff_capteur[7] = 100;
	}
}


//retourne le coefficient de capteur associé
int get_coeff_capteur(uint8_t i){
	return coeff_capteur[i];
}

