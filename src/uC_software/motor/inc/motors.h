
#ifndef MOTORS_H
#define MOTORS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stm32f10x.h>


#define DIR_FORWARD 0
#define DIR_REVERSE 1

#define SUCCESS 					0
#define ERROR_MOTOR_NOT_EXISTING 	1
#define ERROR_DIR_INAVAILABLE		2

typedef struct {
  uint16_t IN_A_PIN, IN_B_PIN, EN_A_PIN, EN_B_PIN, PWM_PIN;
  GPIO_TypeDef IN_A_PORT, IN_B_PORT, EN_A_PORT, EN_B_PORT, PWM_PORT;

  TIM_TypeDef TIM_INSTANCE;
  uint32_t TIM_CHANNEL;
} dc_motor;


void MOTOR_Init(void);
uint8_t motor_set_speed(int motor_id, uint16_t speed, int dir);

#ifdef __cplusplus
}
#endif



#endif // MOTORS_H