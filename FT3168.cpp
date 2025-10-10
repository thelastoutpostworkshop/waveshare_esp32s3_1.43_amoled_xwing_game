// Capacitive Touch controller FT3168 functions
//
#include "board_config.h"
#include "FT3168.h"
#include <Wire.h>

void Touch_Init(void)
{
  static bool s_wireInitialized = false;
  if (!s_wireInitialized)
  {
    Wire.begin(PIN_NUM_TOUCH_SDA, PIN_NUM_TOUCH_SCL, I2C_FREQUENCY);
    s_wireInitialized = true;
  }

  uint8_t data = 0x00;
  I2C_writr_buff(I2C_ADDR_FT3168,0x00,&data,1); //Switch to normal mode

}
uint8_t getTouch(uint16_t *x,uint16_t *y)
{
  uint8_t data;
  uint8_t buf[4];
  if (I2C_read_buff(I2C_ADDR_FT3168,0x02,&data,1) != 0)
    return 0;
  if(data)
  {
    if (I2C_read_buff(I2C_ADDR_FT3168,0x03,buf,4) != 0)
      return 0;
    *x = (((uint16_t)buf[0] & 0x0f)<<8) | (uint16_t)buf[1];
    *y = (((uint16_t)buf[2] & 0x0f)<<8) | (uint16_t)buf[3];
    if(*x > DISPLAY_WIDTH)
    *x = DISPLAY_WIDTH;
    if(*y > DISPLAY_HEIGHT)
    *y = DISPLAY_HEIGHT;
    return 1;
  }
  return 0;
}


