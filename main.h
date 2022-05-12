#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		25
#define ROTATION_COEFF			7
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			30000.0f
#define MAX_DISTANCE 			30.0f
#define ERROR_THRESHOLD			0.01	//[m] because of the noise of the camera
#define KP						20000
#define KI 						5.0f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

#ifdef __cplusplus
}
#endif

#endif
