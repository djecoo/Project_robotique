#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

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
	ARRET,
	DROIT,
	GAUCHE

} DIRECTION_ROBOT;

// returnn the mode of use
uint8_t get_mode(void);

//return the angle
int get_line_position(void);

DIRECTION_ROBOT calcul_direction(int vit_droit, int vit_gauche );

void m_a_j_coeff_catpeurs(DIRECTION_ROBOT direction);

int calcul_moyenne(MICRO_NAME micro);


//cette fonction permet de definir la provenance du bruit en fonction des valeurs des 4 micros

void calcul_angle(int max_right,int max_left,int max_front,int max_back);




//get the norm and the max of each buffer after the fft
maximum_fft get_max_norm_index(BUFFER_NAME_t name);

void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

/*
 * Transfer the coeff of every captor
 */

int get_coeff_capteur(uint8_t i);
/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

int get_max_front_moyenne(void);

double calcul_distance(int32_t intensite_moyenne, int32_t puissance_source);

#endif /* AUDIO_PROCESSING_H */
