//*********************************************************************
//* Library for driving a BMP280 temp/pressure sensor using an Attiny *
//* class device.                                                     *
//*                                                                   *
//* This library implements an I2C driven BMP280 using the TinyWireM  *
//* "software" I2C implementation. It does not (yet) implements the   *
//* SPI driven device.                                                *
//*                                                                   *
//* Based on the ADAFRUIT_BMP280 library.                             *
//* Adapted by Jordi Guillaumes Pons (jguillaumes@gmail.com)          *
//*                                                                   *
//* The following text is part of the original Adafruit version:      *
//* Adafruit invests time and resources providing this open source    *
//* code, please support Adafruit andopen-source hardware by          *
//* purchasing products from Adafruit!                                *
//*                                                                   *
//*  Written by Kevin Townsend for Adafruit Industries.               *
//*  BSD license, all text above must be included in any              *
//*  redistribution                                                   *
//*********************************************************************

#ifndef _TinyBMP280_H_
#define _TinyBMP280_H_
#include "Arduino.h"
//add your includes for the project TinyBMP280 here



namespace tbmp280 {
/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define BMP280_ADDRESS                (0x76)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
enum
{
  BMP280_REGISTER_DIG_T1              = 0x88,
  BMP280_REGISTER_DIG_T2              = 0x8A,
  BMP280_REGISTER_DIG_T3              = 0x8C,

  BMP280_REGISTER_DIG_P1              = 0x8E,
  BMP280_REGISTER_DIG_P2              = 0x90,
  BMP280_REGISTER_DIG_P3              = 0x92,
  BMP280_REGISTER_DIG_P4              = 0x94,
  BMP280_REGISTER_DIG_P5              = 0x96,
  BMP280_REGISTER_DIG_P6              = 0x98,
  BMP280_REGISTER_DIG_P7              = 0x9A,
  BMP280_REGISTER_DIG_P8              = 0x9C,
  BMP280_REGISTER_DIG_P9              = 0x9E,

  BMP280_REGISTER_CHIPID             = 0xD0,
  BMP280_REGISTER_VERSION            = 0xD1,
  BMP280_REGISTER_SOFTRESET          = 0xE0,

  BMP280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

  BMP280_REGISTER_CONTROL            = 0xF4,
  BMP280_REGISTER_CONFIG             = 0xF5,
  BMP280_REGISTER_PRESSUREDATA       = 0xF7,
  BMP280_REGISTER_TEMPDATA           = 0xFA,
};

#define BMP280_CONTROL_VALUE 0x3f // 001 111 11 => temp x1, IIR x16, mode normal

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
    typedef struct
    {
      uint16_t dig_T1;
      int16_t  dig_T2;
      int16_t  dig_T3;

      uint16_t dig_P1;
      int16_t  dig_P2;
      int16_t  dig_P3;
      int16_t  dig_P4;
      int16_t  dig_P5;
      int16_t  dig_P6;
      int16_t  dig_P7;
      int16_t  dig_P8;
      int16_t  dig_P9;

      uint8_t  dig_H1;
      int16_t  dig_H2;
      uint8_t  dig_H3;
      int16_t  dig_H4;
      int16_t  dig_H5;
      int8_t   dig_H6;
    } bmp280_calib_data;
/*=========================================================================*/


class TinyBMP280 {
public:
    TinyBMP280(void);

    bool  begin(uint8_t addr = BMP280_ADDRESS);
    float readTemperature(void);
    float readPressure(void);
    float readAltitude(float seaLevelhPa = 1013.25);

private:
    void readCoefficients(void);
    uint8_t spixfer(uint8_t x);

    void      write8(byte reg, byte value);
    uint8_t   read8(byte reg);
    uint16_t  read16(byte reg);
    uint32_t  read24(byte reg);
    int16_t   readS16(byte reg);
    uint16_t  read16_LE(byte reg); // little endian
    int16_t   readS16_LE(byte reg); // little endian

    uint8_t   _i2caddr;
    int32_t   _sensorID;
    int32_t   t_fine;

    bmp280_calib_data _bmp280_calib;
};


} // End namespace

#endif /* _TinyBMP280_H_ */
