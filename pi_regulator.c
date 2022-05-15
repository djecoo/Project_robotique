


#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <audio_processing.h>
#include <arm_math.h>

#include <capture_ir.h>

static DIRECTION_ROBOT origine_obstacle = ARRET;

void set_origine_obstacle(uint8_t);

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
	//the sound detection is very noisy
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

	speed = KP * error; //KI * sum_error;

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
    int32_t puissance_source = 0;
    double distance_source = 0;



    while(1){

        time = chVTGetSystemTime();


        puissance_moyenne = (calcul_moyenne(FRONT)+calcul_moyenne(LEFT)+calcul_moyenne(BACK)+calcul_moyenne(RIGHT))/4;


        // calcul puissance si il y a une source et si pas identifiee, à moduler
        if(puissance_moyenne > 20000 && puissance_source == 0){
        	puissance_source = puissance_moyenne*4*PI*DIST_REF*DIST_REF;
        }else if(puissance_moyenne < 20000){
        	puissance_source = 0;
        }

        //calcule la distance à partir de la source
        if(puissance_source > 0){
        	distance_source = calcul_distance(puissance_moyenne, puissance_source);
        }

        //computes the speed to give to the motors
        speed = pi_regulator(distance_source, DIST_REF);


        //computes a correction factor to let the robot rotate to be in front of the the sound source
        int angle = get_line_position();

        if (angle >180)
        	angle = (angle - 360);
        speed_correction = angle;

        //these line can be useful depending of the parameter of the environnment
       /* if(abs(speed_correction) < ROTATION_THRESHOLD){
                speed_correction = 0;
         }*/


        switch (get_mode())
        {

        	case 0 : //motor off
        		right_motor_set_speed(0);
        		left_motor_set_speed(0);
        		break;
        	case 1 :	//the motor follows the sound with rotation
        		right_motor_set_speed(-ROTATION_COEFF * speed_correction);
        		left_motor_set_speed(ROTATION_COEFF * speed_correction);
        		break;

        	case 2:	//the motor follows the sound with pid
        		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
        		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
        		break;

        	case 3: ; //the motor follows the sounds and avoid the obstacles

				DIRECTION_ROBOT direction = calcul_direction(speed - ROTATION_COEFF * speed_correction,speed + ROTATION_COEFF * speed_correction);

				m_a_j_coeff_catpeurs(direction);


				int* bufferProxy = get_proxy_buffer_ptr();
				for(uint8_t i = 0; i<=8 ;i++){
					bufferProxy[i] =bufferProxy[i]*(get_coeff_capteur(i)/100);
				 }

				int max_buffer =0;
				unsigned int norme_max_buffer = 0;
				for(uint8_t i =0; i<8;i++){
					if(bufferProxy[i] > max_buffer){
						max_buffer = bufferProxy[i];
						norme_max_buffer = i;
					}
				}

				//determine where the proxy detects the highest value
				set_origine_obstacle(norme_max_buffer);


				//parametre roation proxy entre 1 et 16 -> determine l'urgence de la rotation
				correction_proxy = (1+(((max_buffer-VALEUR_DETECTION_CHOC)*16)/(3920-VALEUR_DETECTION_CHOC)));

				if(max_buffer <VALEUR_DETECTION_CHOC){ //pas de correction
					right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
					left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

				}else if(max_buffer>=VALEUR_DETECTION_CHOC){ //le robot evite l'obstacle en fonction du facteur de rotation_proxy
					right_motor_set_speed(-ROTATION_COEFF * speed_correction*correction_proxy);
					left_motor_set_speed(ROTATION_COEFF * speed_correction*correction_proxy);

				}
					break;
				default :
					right_motor_set_speed(0);
					left_motor_set_speed(0);
					break;
        }
        chThdSleepUntilWindowed(time, time + MS2ST(10));

    }
}

void set_origine_obstacle(uint8_t i){
	if(i == 0){
		origine_obstacle = DEVANT;

	}else if(i == 1){
		origine_obstacle = DEVANT_DROIT;
	}else if(i==2){
		origine_obstacle = DROIT;
	}else if(i ==3){
		origine_obstacle = DERRIERE_DROIT;
	}else if (i ==4){
		origine_obstacle = DERRIERE_GAUCHE;
	}else if(i==5){
		origine_obstacle = GAUCHE;
	}else if(i == 6){
		origine_obstacle = DEVANT_GAUCHE;
	}else if(i == 7){
		origine_obstacle = 0;
	}else{
		origine_obstacle =0;
	}

}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

//simple PI regulator implementation
