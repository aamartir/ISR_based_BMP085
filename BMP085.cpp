#include "BMP085.h"
#include "I2C.h"

BMP085::BMP085() {
}

boolean BMP085::init( uint8_t mode ) 
{
  if ( mode > ULTRAHIGHRES ) 
    mode = ULTRAHIGHRES;
    
  oversampling = mode;

  if( I2C::read8( DEV_ADDR, 0xD0 ) != 0x55 ) 
    return false;

  /* read calibration data */
  ac1 = I2C::read16( DEV_ADDR, CAL_AC1 );
  ac2 = I2C::read16( DEV_ADDR, CAL_AC2 );
  ac3 = I2C::read16( DEV_ADDR, CAL_AC3 );
  ac4 = I2C::read16( DEV_ADDR, CAL_AC4 );
  ac5 = I2C::read16( DEV_ADDR, CAL_AC5 );
  ac6 = I2C::read16( DEV_ADDR, CAL_AC6 );

  b1 = I2C::read16( DEV_ADDR, CAL_B1 );
  b2 = I2C::read16( DEV_ADDR, CAL_B2 );

  mb = I2C::read16( DEV_ADDR, CAL_MB );
  mc = I2C::read16( DEV_ADDR, CAL_MC );
  md = I2C::read16( DEV_ADDR, CAL_MD );
  
  pinMode( EOC_PIN, INPUT );
  //DDRD &= ~(1 << 4);
    
  /* Enable pin change interrupt on Digital Pin 4 (PD4) on any RAISING edge */
  PCICR |= _BV( PCIE2 ); //(1 << PCIE2);
  PCMSK2 |= _BV( PCINT20 ); //(1 << PCINT20); /* PD4 */
  //MCUCR = _BV(ISC01) | _BV(ISC00); /* Rising edge trigger interrupt */

  /* Configure state-machine initial states */
  status.dataState = IDLE_STATE;
  status.eocReceived = 0;
  status.pressureCalibrated = 0;
  status.seaLevelPressure = 101325.0;
  status.counter = 0;
  status.busy = 0;
  
  return true;
}

/* Calculate compensated pressure */
int32_t BMP085::calculateRealPressure( int32_t rawTemp, int32_t rawPressure )
{    
  int32_t UT, UP, B3, B5, B6, X1, X2, X3, p, t;
  uint32_t B4, B7;
  
  // Initial values
  UT = rawTemp;
  UP = rawPressure;

#if BMP085_DEBUG == 1
  // use datasheet numbers!
  UT = 27898;
  UP = 23843;
  ac6 = 23153;
  ac5 = 32757;
  mc = -8711;
  md = 2868;
  b1 = 6190;
  b2 = 4;
  ac3 = -14383;
  ac2 = -72;
  ac1 = 408;
  ac4 = 32741;
  oversampling = 0;
#endif

  // do temperature calculations
  X1 = ((UT - (int32_t)ac6) * (int32_t)ac5) >> 15;
  X2 = ((int32_t)mc << 11)/(X1 + md);
  B5 = X1 + X2;
  t = (B5 + 8) >> 4;
 
#if BMP085_DEBUG == 1
  Serial.print(X1); Serial.print(" ");
  Serial.print(X2); Serial.print(" ");
  Serial.print(B5); Serial.print(" ");
  Serial.println(t); 
#endif

  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ( (B6 * B6) >> 12 )) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = (((((int32_t)ac1 << 2) + X3) << oversampling) + 2) >> 2; //((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;

#if BMP085_DEBUG == 1
  Serial.print(B6); Serial.print(" ");
  Serial.print(X1); Serial.print(" ");
  Serial.print(X2); Serial.print(" ");
  Serial.print(X3); Serial.print(" ");
  Serial.println(B3); 
#endif

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

#if BMP085_DEBUG == 1
  Serial.print(X1); Serial.print(" ");
  Serial.print(X2); Serial.print(" ");
  Serial.print(X3); Serial.print(" ");
  Serial.print(B4); Serial.print(" ");
  Serial.println(B7); 
#endif

  if (B7 < 0x80000000) 
  {
    p = (B7 << 1) / B4; //(B7 * 2) / B4;
  } 
  else 
  {
    p = (B7 << 1) / B3; //(B7 * 2) / B3;
  }

#if BMP085_DEBUG == 1
  Serial.println(p); 
#endif

  X1 = (p >> 8) * (p >> 8);
  
#if BMP085_DEBUG == 1
  Serial.println(X1); 
#endif

  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

  p = p + ((X1 + X2 + (int32_t)3791) >> 4);
  
#if BMP085_DEBUG == 1
  Serial.print(X1); Serial.print(" ");
  Serial.print(X2); Serial.print(" ");
  Serial.println(p); 
#endif

  return p;
}

float BMP085::calculateRealTemperature( int32_t rawTemp )
{
  int32_t UT, X1, X2, B5;     // following ds convention
  float temp;

  UT = rawTemp;

#if BMP085_DEBUG == 1
  // use datasheet numbers!
  UT = 27898;
  ac6 = 23153;
  ac5 = 32757;
  mc = -8711;
  md = 2868;
#endif

  // step 1
  X1 = ((UT - (int32_t)ac6) * (int32_t)ac5) >> 15;
  X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
  B5 = X1 + X2;
  temp = (B5 + 8) >> 4;
  temp /= 10;

  return temp;
}

float BMP085::calculateAltitude( float sealevelPressure, int32_t currentPressure )
{
  float altitude;

  //#ifdef SMOOTH_ALTITUDE_DATA
  //  status.altitude = SMOOTH_FACTOR * status.altitude + (1 - SMOOTH_FACTOR) * (44330 * (1.0 - pow( status.finalPressure / sealevelPressure, 0.1903 )));
  //#else
      altitude = 44330 * (1.0 - pow( currentPressure / sealevelPressure, 0.1903 ));
  //#endif

  return altitude;
}

/* This function will act as a state machine. It will return true, if there's new altitude
 * data available. In main loop, if this function returns true, user can call getAltitude() function */
boolean BMP085::acquire( void )
{
  if( status.eocReceived )
  {
    eocEvent();
    status.dataState++;
  }

  if( !status.busy )
  {
    if( status.dataState == IDLE_STATE )
      status.dataState = GET_RAW_TEMP_STATE;

    if( status.dataState == GET_RAW_TEMP_STATE )
      readRawTempReq();
    else if( status.dataState == GET_RAW_PRESSURE_STATE )
      readRawPressureReq();
    else if( status.dataState == GET_ALTITIDE_STATE )
    {
      /* Calculate altitude based on sea level pressure */
      status.altitude = calculateAltitude( status.seaLevelPressure, status.compensatedPressure );
      status.dataState = IDLE_STATE;
      return true;
    }
  }
  return false;
}

boolean BMP085::calibratePressureAtSeaLevel( void )
{
  if( status.eocReceived )
  {
    eocEvent();
    status.dataState++;
  }
 
  if( !status.busy )
  {
    if( status.dataState == IDLE_STATE )
    {
      status.counter = 0;
      status.pressureCalibrated = 0;
      status.seaLevelPressure = 0;

      status.dataState = GET_RAW_TEMP_STATE;
    }
    
    if( status.dataState == GET_RAW_TEMP_STATE )
      readRawTempReq();
    else if( status.dataState == GET_RAW_PRESSURE_STATE )
      readRawPressureReq();
    else if( status.dataState == GET_ALTITIDE_STATE )
    {
      /* Accumulate pressure */
      status.seaLevelPressure += status.compensatedPressure;
      status.counter++;

      if( status.counter >= CALIBRATION_COUNT )
      {
        status.seaLevelPressure /= CALIBRATION_COUNT;
        status.pressureCalibrated = 1;
        status.dataState = IDLE_STATE;

        return true;
      }
    }
  }

  return false;
}

boolean BMP085::pressureCalibrated( void )
{
  return status.pressureCalibrated;
}

void BMP085::setEOCReceived( void )
{
  status.eocReceived = 1;
}

boolean BMP085::readRawTempReq( void )
{
  if( !status.busy )
  {
    status.dataState = GET_RAW_TEMP_STATE;
    status.busy = 1;
    I2C::write8( DEV_ADDR, CONTROL_REG, READTEMP_CMD );
    
    return true;
  }
  
  return false;
}

boolean BMP085::readRawPressureReq( void )
{
  if( !status.busy )
  {
    status.dataState = GET_RAW_PRESSURE_STATE;
    status.busy = 1;
    I2C::write8( DEV_ADDR, CONTROL_REG, READPRESSURE_CMD + (oversampling << 6) );
    
    return true;
  }
  
  return false;
}

uint8_t BMP085::getState( void )
{
  return status.dataState;
}

float BMP085::getCurrentAltitude( void )
{
  return status.altitude;
}

void BMP085::setState( uint8_t newState )
{
  status.dataState = newState;
}

/* After read request was submitted, End Of Conversion event will trigger */
void BMP085::eocEvent( void )
{
  int32_t val;
  int32_t p;

  switch( status.dataState )
  {
    case GET_RAW_TEMP_STATE:
      status.rawTemperature = I2C::read16( DEV_ADDR, TEMPERATURE_REG );
      break;
    case GET_RAW_PRESSURE_STATE:
      val = I2C::read16( DEV_ADDR, PRESSURE_REG );
      val <<= 8;
      val |= I2C::read8( DEV_ADDR, PRESSURE_REG + 2 );
      val >>= (8 - oversampling);
      status.rawPressure = val;

      /* Calculate compensated pressure */
      status.compensatedPressure = calculateRealPressure( status.rawTemperature, status.rawPressure );
      break;
  }

  /* Have to clear eoc flag */
  status.eocReceived = 0;
  status.busy = 0;
}
















