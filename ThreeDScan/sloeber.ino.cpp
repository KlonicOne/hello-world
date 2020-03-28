#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2019-02-22 20:58:51

#include "Arduino.h"
#include <Arduino_FreeRTOS.h>
#include <Servo.h>
#include <U8glib.h>
#include <Servo.h>

void setup () ;
void loop () ;
void taskGetDistanceMeasurement (void *pvParameters)       ;
void taskDisplayManager (void *pvParameters) ;
void taskServoControl (void *pvParameters)       ;
void draw (void) ;
void setServoPosAngle_deg (int angle_deg, Servo* p_servo) ;

#include "ThreeDScan.ino"


#endif
