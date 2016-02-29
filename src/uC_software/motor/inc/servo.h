
#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stm32f10x.h>


void SERVO_Init();

void servo_set(uint8_t id, uint8_t val);

#ifdef __cplusplus
}
#endif


#endif // SERVO_H