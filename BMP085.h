#ifndef _BMP085_H
#define _BMP085_H

#include "Arduino.h"
#include "Wire.h"

//#define BMP085_DEBUG      1

/* Device Address */
#define DEV_ADDR          0x77

/* Configuration Parameters */
#define ULTRALOWPOWER     0
#define STANDARD          1
#define HIGHRES           2
#define ULTRAHIGHRES      3

/* Calibration Registers (16 bits each, MSB followed by LSB) */
#define CAL_AC1           0xAA  // MSB
#define CAL_AC2           0xAC  // LSB
#define CAL_AC3           0xAE  // and so on
#define CAL_AC4           0xB0  
#define CAL_AC5           0xB2  
#define CAL_AC6           0xB4  
#define CAL_B1            0xB6  
#define CAL_B2            0xB8  
#define CAL_MB            0xBA  
#define CAL_MC            0xBC  
#define CAL_MD            0xBE  

/* Registers */
#define CONTROL_REG       0xF4 
#define TEMPERATURE_REG   0xF6 /* Temperature data is stored in this register */
#define PRESSURE_REG      0xF6 /* Pressure data is stored in this register */ 

/* Commands */
#define READTEMP_CMD      0x2E
#define READPRESSURE_CMD  0x34

//#define SMOOTH_ALTITUDE_DATA
#ifdef SMOOTH_ALTITUDE_DATA
  #define SMOOTH_FACTOR 0.50 
#endif

#define OFF_STATE                  0x00
#define IDLE_STATE                 0x01
#define GET_RAW_TEMP_STATE         0x02
#define GET_RAW_PRESSURE_STATE     0x03
#define GET_ALTITIDE_STATE         0x04

#define CALIBRATION_COUNT           100
#define EOC_PIN                       4
#define EOC_PIN_MASK      (1 << EOC_PIN)

typedef struct _bmp_state
{
  uint8_t dataState;
  boolean busy;
  volatile boolean eocReceived;

  int32_t rawPressure;
  uint16_t rawTemperature;

  int32_t compensatedPressure;
  boolean pressureCalibrated;
  int32_t seaLevelPressure;
  uint8_t counter;

  float altitude;
  //boolean newDataAvailable;
} BMP085_STATUS;

class BMP085 
{
  public:
    BMP085();
    boolean init( uint8_t mode = ULTRAHIGHRES );  // by default go highres
    float calculateRealTemperature( int32_t rawT );
    int32_t calculateRealPressure( int32_t rawT, int32_t rawP );
    float calculateAltitude( float sealevelPressure = 101325.0, int32_t p = 0 ); // std atmosphere
    float getCurrentAltitude( void );

    /* ISR functions */
    boolean acquire( void );
    void eocEvent( void );
    
    boolean readRawTempReq( void );
    boolean readRawPressureReq( void );
    boolean calibratePressureAtSeaLevel( void );
    boolean pressureCalibrated( void );
    
    uint8_t getState( void );
    void setState( uint8_t newState );
    void setEOCReceived( void );

  private:
    BMP085_STATUS status;

    uint8_t oversampling;

    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;
};

#endif 
