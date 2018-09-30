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



namespace tbmp {
/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define BMP280_ADDRESS                (0x76)
	#define BMP180_ADDRESS                (0x77)
	#define BMP280_CONTROL_VALUE 0x3f // 001 111 11 => temp x1, IIR x16, mode normal


class TinyBMPBase {

public:
	float readTemperature(void);
	virtual float readPressure(void);
	int32_t readIntTemperature(void);
	uint32_t readIntPressure(void);
	float readAltitude(float);

protected:
	void      write8(uint8_t reg, uint8_t value);
	uint8_t   read8(uint8_t reg);
	uint16_t  read16(uint8_t reg);
	uint32_t  read24(uint8_t reg);
	int16_t   readS16(uint8_t reg);
	uint16_t  read16_LE(uint8_t reg); // little endian
	int16_t   readS16_LE(uint8_t reg); // little endian

	uint8_t   _i2caddr;
};

enum {
	BMP180_REGISTER_CAL_AC1            = 0xAA,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_AC2            = 0xAC,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_AC3            = 0xAE,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_AC4            = 0xB0,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_AC5            = 0xB2,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_AC6            = 0xB4,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_B1             = 0xB6,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_B2             = 0xB8,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_MB             = 0xBA,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_MC             = 0xBC,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_MD             = 0xBE,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CHIPID             = 0xD0,
	BMP180_REGISTER_VERSION            = 0xD1,
	BMP180_REGISTER_SOFTRESET          = 0xE0,
	BMP180_REGISTER_CONTROL            = 0xF4,
	BMP180_REGISTER_TEMPDATA           = 0xF6,
	BMP180_REGISTER_PRESSUREDATA       = 0xF6,
	BMP180_REGISTER_READTEMPCMD        = 0x2E,
	BMP180_REGISTER_READPRESSURECMD    = 0x34
};

//+
// BMP180 operating modes
//-
typedef enum
{
  BMP180_MODE_ULTRALOWPOWER          = 0,
  BMP180_MODE_STANDARD               = 1,
  BMP180_MODE_HIGHRES                = 2,
  BMP180_MODE_ULTRAHIGHRES           = 3
} bmp180_mode_t;

typedef struct {
	int16_t  ac1;
	int16_t  ac2;
	int16_t  ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t  b1;
	int16_t  b2;
	int16_t  mb;
	int16_t  mc;
	int16_t  md;
} bmp180_calib_data;

class TinyBMP180 : public TinyBMPBase {
public:
    TinyBMP180(void);

    bool  begin(bmp180_mode_t mode = BMP180_MODE_ULTRAHIGHRES);
    float readTemperature(void);
    float readPressure(void);
    int32_t readIntTemperature(void);
    uint32_t readIntPressure(void);

protected:
    void readCoefficients(void);

private:
    int32_t readRawTemperature(void);
    int32_t readRawPressure(void);
    int32_t computeB5(int32_t ut);

    bmp180_mode_t _bmp180Mode;
    bmp180_calib_data _bmp180_coeffs;
};

enum {
	BMP280_REGISTER_DIG_T1             = 0x88,
	BMP280_REGISTER_DIG_T2             = 0x8A,
	BMP280_REGISTER_DIG_T3             = 0x8C,

	BMP280_REGISTER_DIG_P1             = 0x8E,
	BMP280_REGISTER_DIG_P2             = 0x90,
	BMP280_REGISTER_DIG_P3             = 0x92,
	BMP280_REGISTER_DIG_P4             = 0x94,
	BMP280_REGISTER_DIG_P5             = 0x96,
	BMP280_REGISTER_DIG_P6             = 0x98,
	BMP280_REGISTER_DIG_P7             = 0x9A,
	BMP280_REGISTER_DIG_P8             = 0x9C,
	BMP280_REGISTER_DIG_P9             = 0x9E,

	BMP280_REGISTER_CHIPID             = 0xD0,
	BMP280_REGISTER_VERSION            = 0xD1,
	BMP280_REGISTER_SOFTRESET          = 0xE0,

	BMP280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

	BMP280_REGISTER_CONTROL            = 0xF4,
	BMP280_REGISTER_CONFIG             = 0xF5,
	BMP280_REGISTER_PRESSUREDATA       = 0xF7,
	BMP280_REGISTER_TEMPDATA           = 0xFA,
};

typedef struct {
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

class TinyBMP280 : public TinyBMPBase {
public:
    TinyBMP280(void);
	bool  begin(uint8_t addr = BMP280_ADDRESS);
	float readTemperature(void);
	float readPressure(void);
	int32_t readIntTemperature(void);
	uint32_t readIntPressure(void);

protected:
    void readCoefficients(void);

private:
    int32_t   t_fine;
    bmp280_calib_data _bmp280_calib;
};

} // End namespace

#endif /* _TinyBMP280_H_ */
