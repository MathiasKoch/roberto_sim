#pragma once

#include "UsartWithBuffer.h"

class STM32Hardware {

  using SERIAL_CLASS = UsartWithBuffer;

  public:
    STM32Hardware(SERIAL_CLASS* _com , long baud= 115200){
      com = _com;
      baud_ = baud;
    }

    STM32Hardware()
    {
      com = new SERIAL_CLASS(115200);
      baud_ = 115200;
    }
    STM32Hardware(STM32Hardware& h){
      this->baud_ = h.baud_;
    }

    void setBaud(long baud){
      this->baud_= baud;
    }

    int getBaud(){return baud_;}

    void init(){
      //iostream->begin(baud_);
    }

    int read(){
      if(com->dataAvailable()){
        return com->getch();
      }
      else{
        return -1;
      }
    }

    void write(uint8_t* data, int length){
      for(int i=0; i<length; i++){
        while( !com->TXBuffer_FreeSpace() );
        com->putch(data[i]);
      }
    }

    unsigned long time(){return millis();}

  protected:
    SERIAL_CLASS* com;
    long baud_;
};