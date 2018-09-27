#include <Arduino.h>
#include <TinyWireM.h>
#include "TinyBMP280.h"

using namespace tbmp280;

//+
// Empty constructor
//-
TinyBMP280::TinyBMP280() {}

//+
// Write 8-bit value to the device using I2C
//-
void TinyBMP280::write8(byte reg, byte value)
{
	TinyWireM.beginTransmission((uint8_t)_i2caddr);
	TinyWireM.send((uint8_t)reg);
	TinyWireM.send((uint8_t)value);
	TinyWireM.endTransmission();
}


//+
// Read a 8-bit value using I2C
//-
uint8_t TinyBMP280::read8(byte reg) {
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
uint16_t TinyBMP280::read16(byte reg) {
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
uint16_t TinyBMP280::read16_LE(byte reg) {
	uint16_t temp = read16(reg);
	return (temp >> 8) | (temp << 8);
}

//+
// Read a signed 16 bit value from I2C (big endian order)
//-
int16_t TinyBMP280::readS16(byte reg)
{
  return (int16_t)read16(reg);

}

//+
// Read a signed 16 bit value from I2C (little endian order)
//-
int16_t TinyBMP280::readS16_LE(byte reg)
{
  return (int16_t)read16_LE(reg);

}

//+
// Read a signed 24 bit value from I2C. The value comes in three
// bytes, according to the specs found in the BMP280 datasheet.
//-
uint32_t TinyBMP280::read24(byte reg) {
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
// Read the factory set calibration coefficients
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
uint32_t  TinyBMP280::readIntTemperature(void) {
	int32_t var1, var2;

	int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
	adc_T >>= 4;

	var1  = ((((adc_T>>3) - ((int32_t)_bmp280_calib.dig_T1 <<1))) *
			((int32_t)_bmp280_calib.dig_T2)) >> 11;

	var2  = (((((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1)) *
			((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
			((int32_t)_bmp280_calib.dig_T3)) >> 14;

	t_fine = var1 + var2;

	return (uint32_t) (t_fine * 5 + 128) >> 8;
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
// Take and calibrate an altitude reading (based on pressure and QNH)
//-
float TinyBMP280::readAltitude(float seaLevelhPa) {
  float altitude;

  float pressure = readPressure(); // in Si units for Pascal
  pressure /= 100;

  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return altitude;
}
