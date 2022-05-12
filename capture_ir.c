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

static int buffer_proxy[7];

static THD_WORKING_AREA(waCapture_ir, 256);
static THD_FUNCTION(Capture_ir, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    while(1){
    	 systime_t time;
    	 chThdSleepMilliseconds(1000);
    	 int valeur = 0;
    	 for(uint8_t i=0;i<8;i++){
    		 valeur = get_prox(i);
    		 if(valeur<0)
    			 buffer_proxy[i] = -valeur;
    		 else
    			 buffer_proxy[i] = valeur;
    		 //chprintf((BaseSequentialStream *) &SDU1,"sensor %d  = %d \n", i ,buffer_proxy[i]);
    	 }

    	//

    	chThdSleepUntilWindowed(time, time + MS2ST(40));
    }
}

int* get_proxy_buffer_ptr(){
	return buffer_proxy;
}

void capture_ir_start(){
	chThdCreateStatic(waCapture_ir, sizeof(waCapture_ir), NORMALPRIO, Capture_ir, NULL);
}
