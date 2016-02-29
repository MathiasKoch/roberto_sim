

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include <stm32f10x.h>
#include "pid.h"


#define DEBUG_PRINT(args...) printf(args)


float integral;

float PID(float P, float I, float input, float ref){
	int16_t error = input - ref;
	integral += error;
	return error*P + integral*I;
}