
#include <stm32f10x.h>
#include "encoder.h"


void I2C1_Init(void){
    GPIO_InitTypeDef  GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;

    /* I2C1 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    /* I2C1 SDA and SCL configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*SCL is pin06 and SDA is pin 07 for I2C1*/

    /* I2C1 configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;    // 100 kHz operation
    I2C_Init(I2C1, &I2C_InitStructure);

     /*enable I2C*/
    I2C_Cmd(I2C1,ENABLE);

    //NVIC_EnableIRQ(I2C1_EV_IRQn);
}
#if 0
void I2C1_EV_IRQHandler(void){
  __disable_irq();
  /* Check on EV5 */
  if(I2C_GetITStatus(I2C1, I2C_IT_SB)== SET){
    /* Send slave Address for read */
    I2C_Send7bitAddress(I2C1, slaveAddress<<1, I2C_Direction_Receiver);
    if (NumberOfByteToReceive == 0x03){
      /* Disable buffer Interrupts */
      I2C_ITConfig(I2C1, I2C_IT_BUF , DISABLE);
    }else{    	
      /* Enable buffer Interrupts */
      I2C_ITConfig(I2C1, I2C_IT_BUF , ENABLE);
    }
  }else if(I2C_GetITStatus(I2C1, I2C_IT_ADDR)== SET){
    if (NumberOfByteToReceive == 1){
      I2C_AcknowledgeConfig(I2C1, DISABLE);
    }
    
    /* Clear ADDR Register */
    (void)(I2C1->SR1);
    (void)(I2C1->SR2);  
    if (GenerateStartStatus == 0x00){ 
      if (NumberOfByteToReceive == 1){
         I2C_GenerateSTOP(I2C1, ENABLE);  
      }  
      if (NumberOfByteToReceive == 2){
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Next);
        /* Disable buffer Interrupts */
        I2C_ITConfig(I2C1, I2C_IT_BUF , DISABLE);
      }
    }
  }else if((I2C_GetITStatus(I2C1, I2C_IT_RXNE)== SET)
  				&&(I2C_GetITStatus(I2C1, I2C_IT_BTF)== RESET)){  	

    /* Store I2C received data */
    RxBuffer[Rx_Idx++] = I2C_ReceiveData (I2C1);
    NumberOfByteToReceive--;
    
    if (NumberOfByteToReceive == 0x03){
      /* Disable buffer Interrupts */
      I2C_ITConfig(I2C1, I2C_IT_BUF , DISABLE);
    }
    if (NumberOfByteToReceive == 0x00){    	
      /* Disable Error and Buffer Interrupts */
      I2C_ITConfig(I2C1, (I2C_IT_EVT | I2C_IT_BUF), DISABLE);            
    }
  }else if(I2C_GetITStatus(I2C1, I2C_IT_BTF)== SET){ /* BUSY, MSL and RXNE flags */
    /* if Three bytes remaining for reception */
    if (NumberOfByteToReceive == 3){
      I2C_AcknowledgeConfig(I2C1, DISABLE);
      /* Store I2C received data */
      RxBuffer[Rx_Idx++] = I2C_ReceiveData (I2C1);
      NumberOfByteToReceive--;        
    }else if (NumberOfByteToReceive == 2){           
      I2C_GenerateSTOP(I2C1, ENABLE);    
      
      /* Store I2C received data */
      RxBuffer[Rx_Idx++] = I2C_ReceiveData (I2C1);
      NumberOfByteToReceive--;
      /* Store I2C received data */
      RxBuffer[Rx_Idx++] = I2C_ReceiveData (I2C1);
      NumberOfByteToReceive--;        
      /* Disable Error and Buffer Interrupts */
      I2C_ITConfig(I2C1, (I2C_IT_EVT | I2C_IT_BUF), DISABLE);            
    }else{
      /* Store I2C received data */
      RxBuffer[Rx_Idx++] = I2C_ReceiveData (I2C1);
      NumberOfByteToReceive--;
    }
  } 
  __enable_irq();
}
#endif