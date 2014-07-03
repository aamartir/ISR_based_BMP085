#include "I2C.h"

void I2C::write8( uint8_t devID, uint8_t reg_addr, uint8_t val ) 
{
  Wire.beginTransmission( devID );	// start transmission to device 
  Wire.write( reg_addr );				    // send register address
  Wire.write( val );					      // send value to write
  Wire.endTransmission();			      // end transmission
}

void I2C::write16( uint8_t devID, uint8_t reg_addr, uint16_t val )
{
  I2C::write8( devID, reg_addr, val >> 8 );
  I2C::write8( devID, reg_addr + 1, val & 0xFF );
}

bool I2C::readBytes( uint8_t devID, uint8_t reg_addr, uint8_t bytesToRead, uint8_t *buf )
{
  Wire.beginTransmission(devID);	// start transmission to device 
  Wire.write(reg_addr);				    // send register address
  Wire.endTransmission();

  Wire.beginTransmission(devID);
  Wire.requestFrom( (int)devID, (int)bytesToRead );

  uint8_t i = 0;
  while( Wire.available() )
  {
    buf[i++] = Wire.read();
  }
  
  Wire.endTransmission();

  return ( i == bytesToRead );
}

/* Results of the read are stored in ptr */
uint8_t I2C::read8( uint8_t devID, uint8_t reg_addr )
{
  uint8_t b;
  
  Wire.beginTransmission( devID );	// start transmission to device 
  Wire.write( reg_addr );		// send register address
  Wire.endTransmission();

  Wire.beginTransmission( devID );
  Wire.requestFrom( (int)devID, 1 );   // Request 1 byte from device
  
  b = Wire.read();
  Wire.endTransmission();
  
  return ( b );
}

uint16_t I2C::read16( uint8_t devID, uint8_t reg_addr )
{
  uint8_t b[2];
  
  /* Read 1 word (2 bytes)*/
  if( I2C::readBytes(devID, reg_addr, 2, b) )
    return ( (b[0] << 8) | b[1] );
  
  return false;
}

/* bitNumber is from 0 to N-1, right to left. That is, bit 0 is the 
 * least significant bit. bit 1 the 2nd least significant, and so on. 
 */
bool I2C::readBit( uint8_t devID, uint8_t reg_addr, uint8_t bitNumber )
{
  uint8_t b;
  
  /* Read the contents of reg_addr */
  b = I2C::read8( devID, reg_addr ); 
  return ( (b >> bitNumber) & 1 );
}

void I2C::writeBit( uint8_t devID, uint8_t reg_addr, uint8_t bitNumber, bool val )
{
  uint8_t b;

  /* Read the contents of reg_addr */
  b = I2C::read8( devID, reg_addr ); 

  /* Modify the bit in the register */
  if( val )
    b |= (val << bitNumber);
  else
    b &= ~(val << bitNumber);
  
  /* Write new value to reg_addr */
  I2C::write8( devID, reg_addr, b );	
}

void flipBit( uint8_t devID, uint8_t reg_addr, uint8_t bitNumber )
{
  uint8_t b;
  
  /* Read the contents of reg_addr */
  b = I2C::read8( devID, reg_addr ); 
  
  /* XOR operator */
  b ^= ( 1 << bitNumber );
  
  I2C::write8( devID, reg_addr, b );
}
