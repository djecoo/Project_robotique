


#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <audio_processing.h>


void pi_regulator_start(){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
	return;
}




//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	if(distance){
		error = distance - goal;
	}else{
		error = 0;
	}


	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error;// + KI * sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

		chRegSetThreadName(__FUNCTION__);
	    (void)arg;

	    systime_t time;
	    float correction_proxy = 0;
	    int16_t speed = 0;
	    int16_t speed_correction = 0;
	    int32_t puissance_moyenne = 0;
	    int16_t frequence_max = 0;
	    int32_t puissance_source = 0;
	    double distance_source = 0;



	    while(1){
	        time = chVTGetSystemTime();


	        puissance_moyenne = (calcul_moyenne(FRONT)+calcul_moyenne(LEFT)+calcul_moyenne(BACK)+calcul_moyenne(RIGHT))/4;

	        if(puissance_moyenne > 20000 && puissance_source == 0){ // calcul puissance si il y a une source et si pas identifiee, à moduler
	        	puissance_source = puissance_moyenne*4*PI*DIST_REF*DIST_REF;
	        }else if(puissance_moyenne < 20000){
	        	puissance_source = 0;
	        }

	        if(puissance_source > 0){
	        	distance_source = sqrt(((double)puissance_source)/(((double)puissance_moyenne)*4*3.1415) );
	        	distance_source = calcul_distance(puissance_moyenne, puissance_source);
	        }

	        //computes the speed to give to the motors
	        //distance_cm is modified by the image processing thread
	        speed = pi_regulator(calcul_distance(puissance_moyenne, puissance_source), DIST_REF);

	        //chprintf((BaseSequentialStream *) &SDU1,"speed  = %d, DISTANCE = %lf \n", speed, distance_source);

	        //computes a correction factor to let the robot rotate to be in front of the line
	        int angle = get_line_position();

	        if (angle >180)
	        	angle = (angle - 360);
	        speed_correction = angle;


	        //if the line is nearly in front of the camera, don't rotate
	       /* if(abs(speed_correction) < ROTATION_THRESHOLD){
	        	speed_correction = 0;
	        }*/


	        //chprintf((BaseSequentialStream *) &SDU1,"puissance_moyenne = %d /n", puissance_moyenne);
	        //chprintf((BaseSequentialStream *) &SDU1,"DISTANCE = %lf \n", distance_source);
	        //chprintf((BaseSequentialStream *) &SDU1,"DISTANCE  = %d \n", calcul_distance());
	        //chprintf((BaseSequentialStream *) &SDU1,"calcul_moyenne(FRONT) = %d\n,calcul_moyenne(LEFT) = %d, calcul_moyenne(BACK) = %d, calcul_moyenne(RIGHT) = %d\n",calcul_moyenne(FRONT), calcul_moyenne(LEFT),calcul_moyenne(BACK),calcul_moyenne(RIGHT));

	        DIRECTION_ROBOT direction = calcul_direction(speed - ROTATION_COEFF * speed_correction,speed + ROTATION_COEFF * speed_correction);

	        m_a_j_coeff_catpeurs(direction);


	        int* bufferProxy = get_proxy_buffer_ptr();
	        for(uint8_t i = 0; i<8 ;i++){
	        	bufferProxy[i] *= (coeff_capteur[i]/100);
	         }

	        		if(puissance_moyenne > VALEUR_MINIMUM && direction == ARRET/*&& get_max_norm_index() > MIN_FREQ*/ ) // MODULER VALEUR
	        {
	        	//applies the speed from the PI regulator and the correction for the rotation
	        	right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
	        	left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
	        	// AAAA chprintf((BaseSequentialStream *) &SDU1,"speed_correction = %d \n", speed_correction);



	        }else if (puissance_moyenne > VALEUR_MINIMUM){
	        	int max_buffer =0;
	        	uint8_t norme_max_buffer = 0;
	        	//200 valeur detection choc, 3920 valeur max
	        	for(uint8_t i =0; i<8;i++){
	        		if(bufferProxy[i] > max_buffer){
	        			max_buffer = bufferProxy[i];
	        			norme_max_buffer = i;
	        		}
	        	}
	        	if(max_buffer <VALEUR_DETECTION_CHOC){ //pas de correction
	        		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
	        		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

	        	}else{
	        		//parametre roation proxy entre 1 et 10 -> determine l'urgence de la rotation
	        		correction_proxy = (1+(((max_buffer-VALEUR_DETECTION_CHOC)*10)/(3920-VALEUR_DETECTION_CHOC)));
	        		int moyenne_supp = 0;
	        		int moyenne_inf = 0;
	        		/*//on determine la moyenne des capteurs à droite et à gauche pour déterminer le sens de la rotation->le sens de correction_proxy
	        			if(norme_max_erreur >=2 && moyenne_inf !=0)
	        				moyenne_inf = (bufferProxy[norme_max_buffer-1]+bufferProxy[norme_max_buffer-2])/2;
	        			if(norme_max_erreur <=5 && moyenne_supp!=0)
	        				moyenne_supp = (bufferProxy[norme_max_buffer+1]+bufferProxy[norme_max_buffer+2])/2;
	        			if(norme_max_erreur == 1){
	        				moyenne_inf = (bufferProxy[0]+bufferProxy[7])/2;
	        			}else if(norme_max_erreur == 0){
	        				moyenne_inf = (bufferProxy[6]+bufferProxy[7])/2;
	        			}
	        			if(norme_max_erreur == 7){
	        				moyenne_supp = (bufferProxy[0]+bufferProxy[1])/2;
	        			}else if(norme_max_erreur == 5){
	        				moyenne_suppp = (bufferProxy[0]+bufferProxy[7])/2;
	        			}*/

	        		//Ou alors on determine avec le sens de de speed_correction


	        		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction*correction_proxy);
	        		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction*correction_proxy);
	        	}


	        }else{
	        	left_motor_set_speed(0);
	        	right_motor_set_speed(0);
	        	// AAAAA chprintf((BaseSequentialStream *) &SDU1,"EN PLACE \n");
	        }
	        //100Hz
	        chThdSleepUntilWindowed(time, time + MS2ST(10));
	    }
}




