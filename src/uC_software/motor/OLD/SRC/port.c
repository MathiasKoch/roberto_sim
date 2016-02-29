#ifdef STM32F103xB
#include <stm32f1xx_hal.h>
#else
#include <stm32f0xx_hal.h>
#endif

#include "port.h"

extern SPI_HandleTypeDef hspi1;

// #define DEBUG_SPI

#ifdef STM32F103xB
  #define DWM_IRQn EXTI0_IRQn
  #define DWM_IRQ_PIN GPIO_PIN_0

//#define DWM_IRQn EXTI3_IRQn
//#define DWM_IRQ_PIN GPIO_PIN_3
#else
  #define DWM_IRQn EXTI0_1_IRQn
  #define DWM_IRQ_PIN GPIO_PIN_0
#endif

int writetospi(uint16 headerLength, const uint8 *headerBuffer,
                             uint32 bodylength, const uint8 *bodyBuffer)
{
#ifdef DEBUG_SPI
  int i;
  printf("Write to SPI: [ ");
  for (i=0; i<headerLength; i++)
    printf("%02x ", (unsigned int)headerBuffer[i]);
  printf("] [ ");

  for (i=0; i<bodylength; i++)
    printf("%02x ", (unsigned int)bodyBuffer[i]);
  printf("]\r\n");
#endif

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);

  HAL_SPI_Transmit(&hspi1, (uint8 *)headerBuffer, headerLength, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi1, (uint8 *)bodyBuffer, bodylength, HAL_MAX_DELAY);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
  return 0;
}

int readfromspi(uint16 headerLength,  const uint8 *headerBuffer,
                              uint32 readlength, uint8 *readBuffer)
{
  // volatile int dummy;

#ifdef DEBUG_SPI
  int i;
  printf("Read from SPI: [ ");
  for (i=0; i<headerLength; i++)
    printf("%02x ", (unsigned int)headerBuffer[i]);
  printf("] ");
#endif

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);

  HAL_SPI_Transmit(&hspi1, (uint8 *)headerBuffer, headerLength, HAL_MAX_DELAY);
  // printf("%lu", hspi1.Instance->SR & SPI_FLAG_RXNE);
  // while (hspi1.Instance->SR & SPI_FLAG_RXNE) {
  //   printf("R");
  //   dummy = hspi1.Instance->DR;
  // }
  // dummy;
  // printf("%lu ", hspi1.Instance->SR & SPI_FLAG_RXNE);
  HAL_SPI_Receive(&hspi1, (uint8 *)readBuffer, readlength, HAL_MAX_DELAY);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);

#ifdef DEBUG_SPI
  printf("{ ");
  for (i=0; i<readlength; i++)
    printf("%02x ", (unsigned int)readBuffer[i]);
  printf("}\r\n");
#endif
  return 0;
}

static decaIrqStatus_t irqStatus = 0;

decaIrqStatus_t decamutexon(void)
{
  // if it is enabled, disable the exti interrupt
  // Return the previous state
  int state = irqStatus;

#ifdef DEBUG_SPI
  printf("MUTEX ON\r\n");
#endif

  if (state) {
    NVIC_DisableIRQ(DWM_IRQn);
    irqStatus = 0;
  }

  return state;
}

void decamutexoff(decaIrqStatus_t s)
{
#ifdef DEBUG_SPI
  printf("MUTEX OFF\r\n");
#endif

  if (s) {
    NVIC_EnableIRQ(DWM_IRQn);
    irqStatus = 1;
  }
}

void reset_DW1000(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __GPIOB_CLK_ENABLE();

  // Setting the reset LOW and reset it as INPUT mode

  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);

  Sleep(1);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);

  //GPIO_InitStruct.Pin = GPIO_PIN_12;
  //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  //HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  Sleep(100);
}

void setup_DW1000RSTnIRQ(int enable)
{
/*
  GPIO_InitTypeDef GPIO_InitStruct;

  __GPIOB_CLK_ENABLE();

  if (enable) {
    // Setting the reset LOW and reset it as INPUT mode
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    NVIC_EnableIRQ(EXTI15_10_IRQn);
  } else {
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    NVIC_EnableIRQ(EXTI15_10_IRQn);
  }
*/
}

int port_CheckEXT_IRQ()
{
#ifdef STM32F103xB
  //return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
  return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
#else
  return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
#endif
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
    case DWM_IRQ_PIN:
      do{
          dwt_isr();
      } while(port_CheckEXT_IRQ() != 0); //while IRS line active (ARM can only do edge sensitive interrupts)
      HAL_NVIC_ClearPendingIRQ(DWM_IRQn);
      break;
    case GPIO_PIN_12:
      //instance_notify_DW1000_inIDLE(1);
      break;
    case GPIO_PIN_1:
      printf("BUTTON PUSHED!\r\n");
      HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn);
      //instance_notify_DW1000_inIDLE(1);
      break;
    default:
      break;
  }
}
