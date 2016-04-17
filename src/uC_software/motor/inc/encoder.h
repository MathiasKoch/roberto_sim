
#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stm32f10x.h>


/*#define SLAVE_ADDRESS1 (0x10)
#define SLAVE_ADDRESS2 (0x10 | 0x04)
#define SLAVE_ADDRESS3 (0x10 | 0x08)
#define SLAVE_ADDRESS4 (0x10 | 0x04 | 0x08)*/

//#define RXBUFFERSIZE 4


/*extern __IO uint32_t TimeOut;
extern __IO uint8_t Rx_Idx;
extern __IO uint8_t NumberOfByteToReceive;
extern  uint8_t RxBuffer[];
extern  uint8_t slaveAddress;
extern __IO uint8_t GenerateStartStatus;*/

void I2C1_Init(void);


#ifdef __cplusplus
}
#endif


#endif // ENCODER_H