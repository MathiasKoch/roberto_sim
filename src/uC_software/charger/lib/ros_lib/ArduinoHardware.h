/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_ARDUINO_HARDWARE_H_
#define ROS_ARDUINO_HARDWARE_H_

#include "stm32f10x.h"
#include "stm32_time.h"
#include "led.h"

#define BUFFER_SIZE 1024


volatile uint8_t USART_FIFO[BUFFER_SIZE];
volatile uint32_t USART_CNTIN;
volatile uint32_t USART_CNTOUT;

class ArduinoHardware {
  public:
    ArduinoHardware(USART_TypeDef* io , long baud=230400){
      iostream = io;
      baud_ = baud;
      USART_CNTIN = 0;
      USART_CNTOUT = 0;
    }
    ArduinoHardware()
    {
      /* Leonardo support */
      iostream = USART1;
      baud_ = 230400;
      USART_CNTIN = 0;
      USART_CNTOUT = 0;
    }
    ArduinoHardware(ArduinoHardware& h){
      this->iostream = iostream;
      this->baud_ = h.baud_;
      USART_CNTIN = 0;
      USART_CNTOUT = 0;
    }
  
    void setBaud(long baud){
      this->baud_= baud;
    }
  
    int getBaud(){return baud_;}

    void init(){
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
      USART_Cmd(iostream, ENABLE);  
      /* Baud rate 230400, 8-bit data, One stop bit
      * No parity, Do both Rx and Tx, No HW flow control
      */
      usart1_init_struct.USART_BaudRate = baud_;   
      usart1_init_struct.USART_WordLength = USART_WordLength_8b;  
      usart1_init_struct.USART_StopBits = USART_StopBits_1;   
      usart1_init_struct.USART_Parity = USART_Parity_No ;
      usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
      usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      /* Configure USART1 */
      USART_Init(iostream, &usart1_init_struct);
      /* Enable RXNE interrupt */
      USART_ITConfig(iostream, USART_IT_RXNE, ENABLE);
      /* Enable USART1 global interrupt */
      NVIC_EnableIRQ(USART1_IRQn);
    }

    int read(){
      if(USART_CNTIN == USART_CNTOUT){
        return -1;
      }
      
      int returnVal = USART_FIFO[USART_CNTOUT];
      USART_CNTOUT = (USART_CNTOUT + 1) % BUFFER_SIZE;
      return returnVal;
    }

    void write(uint8_t* data, int length){
      for(int i=0; i<length; i++){
        while( !(USART1->SR & 0x00000040) );
        USART_SendData(iostream, data[i]);
      }
    }

    uint32_t time(){
      return millis();
    }

  protected:
    USART_TypeDef* iostream;
    long baud_;

};



extern "C"{
  void USART1_IRQHandler(void){

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET && USART_CNTIN != (( USART_CNTOUT - 1 + BUFFER_SIZE) % BUFFER_SIZE)){

      USART_FIFO[USART_CNTIN] = (uint8_t)USART_ReceiveData(USART1);
      USART_CNTIN = (USART_CNTIN + 1) % BUFFER_SIZE;
    }else{
      // Should NEVER reach this point!
      (void)USART_ReceiveData(USART1);  // Clear interupt flag by reading byte
    }
  }   
}


#endif
