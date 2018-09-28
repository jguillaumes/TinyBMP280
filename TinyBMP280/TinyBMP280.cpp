#include <Arduino.h>
#include <TinyWireM.h>
#include "TinyBMP280.h"

using namespace tbmp;

//+
// Read a 8-bit value using I2C
//-
uint8_t TinyBMPBase::read8(byte reg) {
	uint8_t value;

	TinyWireM.beginTransmission((uint8_t)_i2caddr);
	TinyWireM.send((uint8_t)reg);
	TinyWireM.endTransmission();
	TinyWireM.requestFrom((uint8_t)_i2caddr, (byte)1);
	value = TinyWireM.receive();

	return value;
}

//+
// Read an unsigned 16 bit value from I2C (big endian order)
//-
uint16_t TinyBMPBase::read16(byte reg) {
	uint16_t value;

	TinyWireM.beginTransmission((uint8_t)_i2caddr);
	TinyWireM.send((uint8_t)reg);
	TinyWireM.endTransmission();
	TinyWireM.requestFrom((uint8_t)_i2caddr, (byte)2);
	value = (TinyWireM.receive() << 8) | TinyWireM.receive();

	return value;
}

//+
// Read an unsigned 16 bit value from I2C (little endian order)
//-
uint16_t TinyBMPBase::read16_LE(byte reg) {
	uint16_t temp = read16(reg);
	return (temp >> 8) | (temp << 8);
}

//+
// Read a signed 16 bit value from I2C (big endian order)
//-
int16_t TinyBMPBase::readS16(byte reg)
{
  return (int16_t)read16(reg);

}

//+
// Read a signed 16 bit value from I2C (little endian order)
//-
int16_t TinyBMPBase::readS16_LE(byte reg)
{
  return (int16_t)read16_LE(reg);

}

//+
// Read a signed 24 bit value from I2C. The value comes in three
// bytes, according to the specs found in the BMP280 datasheet.
//-
uint32_t TinyBMPBase::read24(byte reg) {
	uint32_t value;

	TinyWireM.beginTransmission((uint8_t)_i2caddr);
	TinyWireM.send((uint8_t)reg);
	TinyWireM.endTransmission();
	TinyWireM.requestFrom((uint8_t)_i2caddr, (byte)3);

	value = TinyWireM.receive();
	value <<= 8;
	value |= TinyWireM.receive();
	value <<= 8;
	value |= TinyWireM.receive();

	return value;
}

//+
// Write 8-bit value to the device using I2C
//-
void TinyBMPBase::write8(byte reg, byte value)
{
	TinyWireM.beginTransmission((uint8_t)_i2caddr);
	TinyWireM.send((uint8_t)reg);
	TinyWireM.send((uint8_t)value);
	TinyWireM.endTransmission();
}

//+
// Take and calibrate an altitude reading (based on pressure and QNH)
//-
float TinyBMPBase::readAltitude(float seaLevelhPa) {
  float altitude;

  float pressure = readPressure(); // in Si units for Pascal
  pressure /= 100;

  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return altitude;
}

//+
// Empty constructor
//-
TinyBMP280::TinyBMP280() {}

//+
// Read the factory set calibration coefficients for the BMP280
//-
void TinyBMP280::readCoefficients(void) {
	_bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
	_bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
	_bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

	_bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
	_bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
	_bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
	_bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
	_bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
	_bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
	_bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
	_bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
	_bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);
}

bool TinyBMP280::begin(uint8_t addr) {
	_i2caddr = addr;

	TinyWireM.begin();

	if (read8(BMP280_REGISTER_CHIPID) != 0x58)
		return false;

	readCoefficients();
	write8(BMP280_REGISTER_CONTROL, BMP280_CONTROL_VALUE);
	// 001 111 11 => temp x1, IIR x16, mode normal
	return true;
}

//+
// Take and calibrate a temperature reading
//-
int32_t  TinyBMP280::readIntTemperature(void) {
	int32_t var1, var2;

	int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
	adc_T >>= 4;

	var1  = ((((adc_T>>3) - ((int32_t)_bmp280_calib.dig_T1 <<1))) *
			((int32_t)_bmp280_calib.dig_T2)) >> 11;

	var2  = (((((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1)) *
			((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
			((int32_t)_bmp280_calib.dig_T3)) >> 14;

	t_fine = var1 + var2;

	return (int32_t) (t_fine * 5 + 128) >> 8;
}

float  TinyBMP280::readTemperature(void) {
	float T  = (t_fine * 5 + 128) >> 8;
	return T/100;
}


//+
// Take and calibrate a pressure reading
//-
uint32_t  TinyBMP280::readIntPressure(void) {
  int64_t var1, var2, p;

  // Must be done first to get the t_fine variable set up
  readTemperature();

  int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1*(int64_t)_bmp280_calib.dig_P5)<<17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3)>>8) +
    ((var1 * (int64_t)_bmp280_calib.dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calib.dig_P1)>>33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7)<<4);
  return (uint32_t) (p>>8);
}

float  TinyBMP280::readPressure(void) {
	  int64_t var1, var2, p;

	  // Must be done first to get the t_fine variable set up
	  readTemperature();

	  int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
	  adc_P >>= 4;

	  var1 = ((int64_t)t_fine) - 128000;
	  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
	  var2 = var2 + ((var1*(int64_t)_bmp280_calib.dig_P5)<<17);
	  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4)<<35);
	  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3)>>8) +
	    ((var1 * (int64_t)_bmp280_calib.dig_P2)<<12);
	  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calib.dig_P1)>>33;

	  if (var1 == 0) {
	    return 0;  // avoid exception caused by division by zero
	  }
	  p = 1048576 - adc_P;
	  p = (((p<<31) - var2)*3125) / var1;
	  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
	  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

	  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7)<<4);

	  return (float)p/256;

}

//+
// Empty constructor for the BMP180
//-
TinyBMP180::TinyBMP180() {}


//+
// Initialize the device communications, set work mode and
// read the compensation coefficients.
//-
bool TinyBMP180::begin(bmp180_mode_t mode) {
	// Enable I2C
	_i2caddr = BMP180_ADDRESS;

	TinyWireM.begin();

	/* Mode boundary check */
	if ((mode > BMP180_MODE_ULTRAHIGHRES) || (mode < 0))
	{
		mode = BMP180_MODE_ULTRAHIGHRES;
	}

	/* Make sure we have the right device */
	uint8_t id = read8(BMP180_REGISTER_CHIPID);
	if(id != 0x55)
	{
		return false;
	}

	/* Set the mode indicator */
	_bmp180Mode = mode;

	/* Coefficients need to be read once */
	readCoefficients();

	return true;
}

//+
// Read factory set calibration coefficients for the BMP180
//-
void TinyBMP180::readCoefficients(void) {
	_bmp180_coeffs.ac1 = readS16(BMP180_REGISTER_CAL_AC1);
	_bmp180_coeffs.ac2 = readS16(BMP180_REGISTER_CAL_AC2);
	_bmp180_coeffs.ac3 = readS16(BMP180_REGISTER_CAL_AC3);
	_bmp180_coeffs.ac4 = read16(BMP180_REGISTER_CAL_AC4);
	_bmp180_coeffs.ac5 = read16(BMP180_REGISTER_CAL_AC5);
	_bmp180_coeffs.ac6 = read16(BMP180_REGISTER_CAL_AC6);
	_bmp180_coeffs.b1  = readS16(BMP180_REGISTER_CAL_B1);
	_bmp180_coeffs.b2  = readS16(BMP180_REGISTER_CAL_B2);
	_bmp180_coeffs.mb  = readS16(BMP180_REGISTER_CAL_MB);
	_bmp180_coeffs.mc  = readS16(BMP180_REGISTER_CAL_MC);
	_bmp180_coeffs.md  = readS16(BMP180_REGISTER_CAL_MD);
}

//+
// Read raw (uncompensated) temperature as integer
//-
int32_t TinyBMP180::readRawTemperature() {
    uint16_t t;
    write8(BMP180_REGISTER_CONTROL, BMP180_REGISTER_READTEMPCMD);
    delay(5);
    t = read16(BMP180_REGISTER_TEMPDATA);
    return (int32_t) t;
}

//+
// Read raw (uncompensated) pressure as integer
//-
int32_t TinyBMP180::readRawPressure(void) {
	uint8_t  p8;
	uint16_t p16;
	int32_t  p32;

	write8(BMP180_REGISTER_CONTROL, BMP180_REGISTER_READPRESSURECMD + (_bmp180Mode << 6));
	switch(_bmp180Mode)
	{
	case BMP180_MODE_ULTRALOWPOWER:
		delay(5);
		break;
	case BMP180_MODE_STANDARD:
		delay(8);
		break;
	case BMP180_MODE_HIGHRES:
		delay(14);
		break;
	case BMP180_MODE_ULTRAHIGHRES:
	default:
		delay(26);
		break;
	}

	p16 = read16(BMP180_REGISTER_PRESSUREDATA);
	p32 = (uint32_t)p16 << 8;
	p8  = read8(BMP180_REGISTER_PRESSUREDATA+2);
	p32 += p8;
	p32 >>= (8 - _bmp180Mode);

	return (int32_t) p32;
}

//+
// Compute B5 coefficient used in calculations
//-
int32_t TinyBMP180::computeB5(int32_t ut) {
  int32_t X1 = (ut - (int32_t)_bmp180_coeffs.ac6) * ((int32_t)_bmp180_coeffs.ac5) >> 15;
  int32_t X2 = ((int32_t)_bmp180_coeffs.mc << 11) / (X1+(int32_t)_bmp180_coeffs.md);
  return X1 + X2;
}



//+
// Take and calibrate a temperature reading in integer form
// The value is returned i Celsius * 100 (ie, 2340 = 23.4 C)
//-
int32_t TinyBMP180::readIntTemperature(void) {
	int32_t t, UT, B5;

	UT = readRawTemperature();

	B5 = computeB5(UT);
	t = (B5+8) >> 4;
	t *= 10;
	return t;
}

//+
// Take and calibrate a temperature reading in float form
// The value returned is in celsius
//-
float  TinyBMP180::readTemperature(void) {
	int32_t UT, B5;
	float t;

	UT = readRawTemperature();
	B5 = computeB5(UT);
	t = (B5+8) >> 4;
	t /= 10;
	return t;
}


//+
// Take and calibrate a pressure reading
//-
uint32_t  TinyBMP180::readIntPressure(void) {
	int32_t  ut = 0, up = 0, compp = 0;
	int32_t  x1, x2, b5, b6, x3, b3, p;
	uint32_t b4, b7;

	/* Get the raw pressure and temperature values */
	ut = readRawTemperature();
	up = readRawPressure();

	/* Temperature compensation */
	b5 = computeB5(ut);

	/* Pressure compensation */
	b6 = b5 - 4000;
	x1 = (_bmp180_coeffs.b2 * ((b6 * b6) >> 12)) >> 11;
	x2 = (_bmp180_coeffs.ac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = (((((int32_t) _bmp180_coeffs.ac1) * 4 + x3) << _bmp180Mode) + 2) >> 2;
	x1 = (_bmp180_coeffs.ac3 * b6) >> 13;
	x2 = (_bmp180_coeffs.b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (_bmp180_coeffs.ac4 * (uint32_t) (x3 + 32768)) >> 15;
	b7 = ((uint32_t) (up - b3) * (50000 >> _bmp180Mode));

	if (b7 < 0x80000000)
	{
		p = (b7 << 1) / b4;
	}
	else
	{
		p = (b7 / b4) << 1;
	}

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	compp = p + ((x1 + x2 + 3791) >> 4);

	return (uint32_t) compp;
}

float  TinyBMP180::readPressure(void) {
	return (float) readIntPressure();
}


