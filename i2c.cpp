// I2C helper functions implemented with Arduino Wire (new I2C driver)
//
#include "i2c.h"
#include <Wire.h>

extern "C" {

uint8_t I2C_writr_buff(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (len && buf)
  {
    Wire.write(buf, len);
  }
  return Wire.endTransmission();
}

uint8_t I2C_read_buff(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err)
    return err;

  uint8_t count = Wire.requestFrom(static_cast<int>(addr), static_cast<int>(len));
  if (count != len)
    return 1;

  for (uint8_t i = 0; i < len; ++i)
  {
    if (Wire.available())
      buf[i] = Wire.read();
  }
  return 0;
}

} // extern "C"

uint8_t I2C_master_write_read_device(uint8_t addr, uint8_t *writeBuf, uint8_t writeLen, uint8_t *readBuf, uint8_t readLen)
{
  Wire.beginTransmission(addr);
  if (writeLen && writeBuf)
  {
    Wire.write(writeBuf, writeLen);
  }
  uint8_t err = Wire.endTransmission(readLen > 0 ? false : true);
  if (err)
    return err;

  if (readLen && readBuf)
  {
    uint8_t count = Wire.requestFrom(static_cast<int>(addr), static_cast<int>(readLen));
    if (count != readLen)
      return 1;
    for (uint8_t i = 0; i < readLen; ++i)
    {
      if (Wire.available())
        readBuf[i] = Wire.read();
    }
  }
  return 0;
}
