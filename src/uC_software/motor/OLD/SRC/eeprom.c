#include <stm32f0xx_hal.h>
#include <stdio.h>

#include <stdbool.h>
#include <string.h>

#include "eeprom.h"

char* content = "Bitcraze";

static I2C_HandleTypeDef * hi2c;
static int devAddr = 0xA0;

void eepromInit(I2C_HandleTypeDef * i2c)
{
  hi2c = i2c;
}

bool eepromTest()
{
  bool pass = true;
  static char buffer[10];

  pass &= eepromRead(0, buffer, strlen(content));
  buffer[strlen(content)] = 0;

  if (strcmp(content, buffer) != 0) {
    pass &= eepromWrite(0, content, strlen(content));

    HAL_Delay(10);

    pass &= eepromRead(0, buffer, strlen(content));
    buffer[strlen(content)] = 0;

    if (strcmp(content, buffer) != 0) {
      pass = false;
    }
  }

  return pass;
}

bool eepromRead(int address, void* data, size_t length)
{
  int status;

  status = HAL_I2C_Mem_Read(hi2c, devAddr, address, I2C_MEMADD_SIZE_16BIT, data, length, 100);

  if (status == HAL_OK)
    return true;

  return false;
}

bool eepromWrite(int address, void* data, size_t length)
{
  int status;

  status = HAL_I2C_Mem_Write(hi2c, devAddr, address, I2C_MEMADD_SIZE_16BIT, data, length, 100);

  if (status == HAL_OK)
    return true;

  return false;
}
