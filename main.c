#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <leds.h>
#include <i2c.h>
#include<sensors/proximity.h>
#include<msgbus/messagebus.h>
#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
#include <capture_ir.h>
#include <pi_regulator.h>



//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

messagebus_t bus;

//initialise bus pour ir ??-> a verifier
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);



static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

int main(void)
{
	/*int zero_right = 0;
	int zero_left = 0;
	int negatif_right =0;
	int negatif_left = 0;*/
	 volatile uint16_t time_fft = 0;

    halInit();
    chSysInit();
    mpu_init();





    messagebus_init(&bus, &bus_lock, &bus_condvar);
    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    timer12_start();
    //inits the motors
    motors_init();

    //start the pi regulator
    pi_regulator_start();

    //start the proximity sensor
     proximity_start();

    //calibrate the proximity sensor with the ambient light
    calibrate_ir();

    //start the ir capture
    capture_ir_start();


    //temp tab used to store values in complex_float format
    //needed bx doFFT_c
    static complex_float temp_tab_right[FFT_SIZE];
    //static complex_float temp_tab_left[FFT_SIZE];

    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it
    static float send_tab[FFT_SIZE];

#ifdef SEND_FROM_MIC
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
#endif  /* SEND_FROM_MIC */

    /* Infinite loop. */
    //wait 1 sec before sending the reference
    chThdSleepMilliseconds(1000);
    int front_mic = get_max_front_moyenne();
    echantillone_distance(front_mic);


    while (1) {


    	GPTD12.tim->CNT = 0;
#ifdef SEND_FROM_MIC
        //waits until a result must be sent to the computer
        wait_send_to_computer();





       // float* bufferCmplxInputRight = get_audio_buffer_ptr(RIGHT_CMPLX_INPUT);
        //float* bufferCmplxInputLeft = get_audio_buffer_ptr(LEFT_CMPLX_INPUT);

        /*for(uint16_t i = 0 ; i < (2*FFT_SIZE) ; i+=2){
        	temp_tab_right[i/2].real = bufferCmplxInputRight[i];
            temp_tab_right[i/2].imag = bufferCmplxInputRight[i+1];
            temp_tab_left[i/2].real = bufferCmplxInputLeft[i];
            temp_tab_left[i/2].imag = bufferCmplxInputLeft[i+1];
            }
        for(uint16_t i = 0 ; i < (2*FFT_SIZE) ; i+=2){

        	if(temp_tab_right[i/2].real == 0)
        		zero_right++;
			if(temp_tab_left[i/2].real == 0)
				zero_left++;
			if(temp_tab_left[i/2].real <0)
				negatif_left++;
			if(temp_tab_right[i/2].real <0)
				negatif_right++;


        	chprintf((BaseSequentialStream *) &SDU1, "Right real = %d , Right imaginaire = %d \n",temp_tab_right[i/2].real, temp_tab_right[i/2].imag);
        	chprintf((BaseSequentialStream *) &SDU1, "Left real = %d , Left imaginaire = %d \n",temp_tab_left[i/2].real, temp_tab_left[i/2].imag);
        }

        chprintf((BaseSequentialStream *) &SDU1, "zero_right = %d , zero_left = %d , negatif_left = %d , negatif_right = %d", zero_right, zero_left,negatif_left,negatif_right);
        chprintf((BaseSequentialStream *) &SDU1,"\n\n\n\n"); */




#ifdef DOUBLE_BUFFERING
        //we copy the buffer to avoid conflicts
        arm_copy_f32(get_audio_buffer_ptr(RIGHT_OUTPUT), send_tab, FFT_SIZE);
        SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, FFT_SIZE);



        //chprintf((BaseSequentialStream *) &SDU1,"max_right  = %d max_norm_index_right = %d\n,max_left = %d max_norm_index_left = %d \n",max_right, norm_right,max_left,norm_left);
        //chprintf((BaseSequentialStream *) &SDU1,"Max_front  = %d max_norm_index_front = %d\n,max_back = %d max_norm_index_back = %d \n\n\n",max_front,norm_front,max_back,norm_back);


        time_fft = GPTD12.tim->CNT;



        //chprintf((BaseSequentialStream *) &SDU1, "time fft = %d us", time_fft);
#else
        SendFloatToComputer((BaseSequentialStream *) &SD3, get_audio_buffer_ptr(LEFT_OUTPUT), FFT_SIZE);
#endif  /* DOUBLE_BUFFERING */

#else
        //time measurement variables
        volatile uint16_t time_fft = 0;
        volatile uint16_t time_mag  = 0;

        float* bufferCmplxInput = get_audio_buffer_ptr(LEFT_CMPLX_INPUT);
        float* bufferOutput = get_audio_buffer_ptr(LEFT_OUTPUT);

        uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, bufferCmplxInput, FFT_SIZE);

        if(size == FFT_SIZE){
            /*
            *   Optimized FFT
            */
            
            chSysLock();
            //reset the timer counter
            GPTD12.tim->CNT = 0;

            doFFT_optimized(FFT_SIZE, bufferCmplxInput);

            time_fft = GPTD12.tim->CNT;
            chSysUnlock();

            /*
            *   End of optimized FFT
            */

            /*
            *   Non optimized FFT
            */

            // //need to convert the float buffer into complex_float struct array
            // for(uint16_t i = 0 ; i < (2*FFT_SIZE) ; i+=2){
            //     temp_tab[i/2].real = bufferCmplxInput[i];
            //     temp_tab[i/2].imag = bufferCmplxInput[i+1];
            // }

            // chSysLock();
            // //reset the timer counter
            // GPTD12.tim->CNT = 0;

            // //do a non optimized FFT
            // doFFT_c(FFT_SIZE, temp_tab);

            // time_fft = GPTD12.tim->CNT;
            // chSysUnlock();
            
            // //reconverts the result into a float buffer
            // for(uint16_t i = 0 ; i < (2*FFT_SIZE) ; i+=2){
            //     bufferCmplxInput[i] = temp_tab[i/2].real;
            //     bufferCmplxInput[i+1] = temp_tab[i/2].imag;
            // }

            /*
            *   End of non optimized FFT
            */

            chSysLock();
            //reset the timer counter
            GPTD12.tim->CNT = 0;

            arm_cmplx_mag_f32(bufferCmplxInput, bufferOutput, FFT_SIZE);

            time_mag = GPTD12.tim->CNT;
            chSysUnlock();

            //SendFloatToComputer((BaseSequentialStream *) &SD3, bufferOutput, FFT_SIZE);
            chprintf((BaseSequentialStream *) &SDU1, "time fft = %d us, time magnitude = %d us\n",time_fft, time_mag);

        }
#endif  /* SEND_FROM_MIC */
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
