/*
 * ir_processing.c
 *
 *  Created on: 12 mai 2022
 *      Author: Jean Cordonnier
 */

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <audio_processing.h>
#include<capture_ir.h>
#include<sensors/proximity.h>

//tableau static pour stocker la valeur des capteurs
static int buffer_proxy[8];

static THD_WORKING_AREA(waCapture_ir, 256);
static THD_FUNCTION(Capture_ir, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time ;
    //va lire les capteurs, prendre leur valeur absolue et la stocker dans buffr_proxy
    while(1)
    {

    	time = chVTGetSystemTime();

    	chThdSleepMilliseconds(1000);
    	int valeur = 0;

    	for(uint8_t i=0;i<=8;i++){
    		valeur = get_prox(i);
    		if(valeur<0)
    			buffer_proxy[i] = -valeur;
    		else
    			buffer_proxy[i] = valeur;
    	 }

    //Plus lent que 2*la fréquence du PID pour eviter de stocker des valeurs qui ne seront pas traitées
    chThdSleepUntilWindowed(time, time + MS2ST(20));
    }
}

//retourne les valeurs lues
int* get_proxy_buffer_ptr(){
	return buffer_proxy;
}

//pour démarrer le thread
void capture_ir_start(){
	chThdCreateStatic(waCapture_ir, sizeof(waCapture_ir), NORMALPRIO, Capture_ir, NULL);
}
