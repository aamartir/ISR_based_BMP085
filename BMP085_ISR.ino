#include <Wire.h>
#include "BMP085.h"
#include "I2C.h"

// EOC ( End of Conversion )
// XCLR is reset pin

BMP085 bmp;
int32_t seaLevelPressure;

void setup() 
{
  Serial.begin(115200);
  Wire.begin();
    
  if ( !bmp.init() ) 
  {
    Serial.println("Error");
    while (1) {}
  }
  
  // Calibrate pressure at sea level
  Serial.print( "Calibrating pressure at current altitude..." );
  while( !bmp.pressureCalibrated() )
    bmp.calibratePressureAtSeaLevel();
  
  Serial.println( "Done" );  
}

void loop() 
{
  if( bmp.acquire() )
    Serial.println( bmp.getCurrentAltitude() );
}

volatile boolean bit_last_state = 0;
ISR( PCINT2_vect )
{
  if( ((PIND & EOC_PIN_MASK) > 0) && bit_last_state == 0 )
  {
    if( bmp.getState() > OFF_STATE )
    {
      bit_last_state = (PIND >> 0x10) & 0x01;
      bmp.setEOCReceived();
    }
  }
}
