#ifndef I2C_H
#define I2C_H

#include <Wire.h>

class I2C
{
  public:
    /* Write functions */
    static void writeBit    ( uint8_t devID, uint8_t reg_addr, uint8_t bitNumber, bool val );
    static void flipBit     ( uint8_t devID, uint8_t reg_addr, uint8_t bitNumber );
    static void write8      ( uint8_t devID, uint8_t reg_addr, uint8_t val );
    static void write16     ( uint8_t devID, uint8_t reg_addr, uint16_t val );
    
    /* Read functions */
    static bool readBit     ( uint8_t devID, uint8_t reg_addr, uint8_t bitNumber );
    static uint8_t  read8    ( uint8_t devID, uint8_t reg_addr );
    static uint16_t read16   ( uint8_t devID, uint8_t reg_addr );
    static bool readBytes   ( uint8_t devID, uint8_t reg_addr, uint8_t bytesToRead, uint8_t *buf );
};

#endif
