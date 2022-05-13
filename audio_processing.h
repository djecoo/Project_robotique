#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024


#define MIN_VALUE_THRESHOLD	10000
#define VALEUR_DETECTION_CHOC 200
#define DISTANCE_REFERENCE 40000 // valeur équivalente à 6cm
#define MIN_FREQ		800	//we don't analyze before this index to not use resources for nothing 500=370hz
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


#define NBR_ECHANTILLON 4
#define NBR_MICRO 4
#define VALEUR_MINIMUM	40000
#define MARGE_ANGLE 0.12 //%
#define VALEUR_MIN_PROXY 200

#define DIST_REF 0.07 //[m]

#define PERCENT

typedef struct maximum_fft{
    int max, norm;
}maximum_fft;

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

typedef enum{
	LEFT = 0,
	RIGHT,
	FRONT,
	BACK
}MICRO_NAME;

typedef enum{
	DEVANT = 0,
	DEVANT_DROIT,
	DERRIERE_DROIT,
	DERRIERE,
	DERRIERE_GAUCHE,
	DEVANT_GAUCHE,
	ARRET



} DIRECTION_ROBOT;

/*
 * Return the index of the maximum after the fft
 */


DIRECTION_ROBOT calcul_direction(int vit_droit, int vit_gauche );

void m_a_j_coeff_catpeurs(DIRECTION_ROBOT direction);

int calcul_moyenne(MICRO_NAME micro);

void echantillone_distance(int valeur);

//cette fonction permet de definir la provenance du bruit en fonction des valeurs des 4 micros

void calcul_angle(int max_right,int max_left,int max_front,int max_back);

void update_moyenne(uint8_t mustSend,int max_right,int max_left,int max_front, int max_back);

void led_direction(int max_right,int max_left,int max_front,int max_back);

maximum_fft get_max_norm_index(BUFFER_NAME_t name);

void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

//int pi_regulator(int max_right, int max_left, int max_front, int max_back);

int get_max_front_moyenne(void);

double calcul_distance(int32_t intensite_moyenne, int32_t puissance_source);



#endif /* AUDIO_PROCESSING_H */
