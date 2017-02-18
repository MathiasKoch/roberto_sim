#pragma once

#include "stm32f10x.h"
#include "stm32_time.h"


const static int UART_BUFSIZE = 1024;
/* @brief Receive buffer. */
volatile uint8_t RX[UART_BUFSIZE];
/* @brief Transmit buffer. */
volatile uint8_t TX[UART_BUFSIZE];
/* @brief Receive buffer head. */
volatile uint16_t RX_Head;
/* @brief Receive buffer tail. */
volatile uint16_t RX_Tail;
/* @brief Transmit buffer head. */
volatile uint16_t TX_Head;
/* @brief Transmit buffer tail. */
volatile uint16_t TX_Tail;


class UsartWithBuffer {

protected:


public:

  /*
   * Use the constructor base initialiser to set up the USART at 57600
   */

  UsartWithBuffer( int _baud = 115200) {
    RX_Tail = 0;
    RX_Head = 0;
    TX_Tail = 0;
    TX_Head = 0;

    USART_InitTypeDef usart1_init_struct;
    /* Bit configuration structure for GPIOA PIN9 and PIN10 */
    GPIO_InitTypeDef gpioa_init_struct;

    /* Enalbe clock for USART1, AFIO and GPIOA */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
                          
    /* GPIOA PIN9 alternative function Tx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_9;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpioa_init_struct);
    /* GPIOA PIN9 alternative function Rx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_10;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpioa_init_struct);

    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);  
    /* Baud rate 230400, 8-bit data, One stop bit
    * No parity, Do both Rx and Tx, No HW flow control
    */
    usart1_init_struct.USART_BaudRate = _baud;   
    usart1_init_struct.USART_WordLength = USART_WordLength_8b;  
    usart1_init_struct.USART_StopBits = USART_StopBits_1;   
    usart1_init_struct.USART_Parity = USART_Parity_No ;
    usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /* Configure USART1 */
    USART_Init(USART1, &usart1_init_struct);
    /* Enable RXNE interrupt */
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    /* Enable USART1 global interrupt */
    NVIC_EnableIRQ(USART1_IRQn);

  }

  bool TXBuffer_Empty()
  {
    /* Make copies to make sure that volatile access is specified. */
    uint16_t tempHead = TX_Head;
    uint16_t tempTail = TX_Tail;
    /* There are data left in the buffer unless Head and Tail are equal. */
    return (tempHead == tempTail);
  }

  bool TXBuffer_FreeSpace()
  {
    /* Make copies to make sure that volatile access is specified. */
    uint16_t tempHead = (TX_Head + 1) & (UART_BUFSIZE-1);
    uint16_t tempTail = TX_Tail;
    /* There are data left in the buffer unless Head and Tail are equal. */
    return (tempHead != tempTail);
  }

  bool putch(uint8_t data)
  {

    uint16_t tempTX_Head;
    bool isFree = TXBuffer_FreeSpace();

    if(isFree)
    {
    	tempTX_Head = TX_Head;

    	__disable_irq();
    	TX[tempTX_Head]= data;
    	/* Advance buffer head. */
    	TX_Head = (tempTX_Head + 1) & (UART_BUFSIZE-1);
    	__enable_irq();

    	/* Enable TXE interrupt. */
    	USART_ITConfig(USART1,USART_IT_TXE,ENABLE);
    }
    return isFree;
  }

  /* Send a string */
  void puts(char *s)
  {
    while (*s) putch(*s++);
  }

  bool dataAvailable(){
    uint16_t tempHead = RX_Head;
    uint16_t tempTail = RX_Tail;
    /* There are data left in the buffer unless Head and Tail are equal. */
    return (tempHead != tempTail);
  }

  uint8_t getch(){
    uint8_t ans;

    __disable_irq();
    ans = (RX[RX_Tail]);
    /* Advance buffer tail. */
    RX_Tail = (RX_Tail + 1) & (UART_BUFSIZE-1);
    __enable_irq();

    return ans;
  }

};


extern "C"{
  void USART1_IRQHandler(void){
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
      /* Advance buffer head. */
      uint16_t tempRX_Head = (RX_Head + 1) & (UART_BUFSIZE-1);

      /* Check for overflow. */
      uint16_t tempRX_Tail = RX_Tail;
      uint8_t data = (uint8_t)USART_ReceiveData(USART1);
      USART_ClearITPendingBit(USART1,USART_IT_RXNE);
      if (tempRX_Head == tempRX_Tail) {
        /* Disable the Receive interrupt */
        USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);
      }else{
        RX[RX_Head] = data;
        RX_Head = tempRX_Head;
      }

    }
    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET){

      /* Check if all data is transmitted. */
      uint16_t tempTX_Tail = TX_Tail;
      if (TX_Head == tempTX_Tail){
        /* Overflow MAX size Situation */
        /* Disable the Transmit interrupt */
        USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
      }else{
        /* Start transmitting. */
        uint8_t data = TX[TX_Tail];
        USART_SendData(USART1, data);
        USART_ClearITPendingBit(USART1, USART_IT_TXE);

        /* Advance buffer tail. */
        TX_Tail = (TX_Tail + 1) & (UART_BUFSIZE-1);
      }
    }
  }   
}