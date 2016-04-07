
#ifndef LED_H
#define LED_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stm32f10x.h>


void LED_Init();

void led_set(uint8_t val);

void DEBUG_Init();
void debug_toggle(void);

void SHUTDOWN_Init(void);

#ifdef __cplusplus
}
#endif


#endif // LED_H