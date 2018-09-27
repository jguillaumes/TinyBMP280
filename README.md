# TinyBMP280: a driver for BMP280 sensor devices under AVR tiny MCUs

This is a version of the ADAFRUIT_BMP280_Library library tailored to run
on AVR tiny devices. It replaces the standard Arduino I2C library by the
TinyWireM one, which uses an USI-based implementation of the I2C protocol.

This version of the library can replace the Adafruit one as a drop-in. The public interface is compatible so very few (if any) code modification will be needed. Please take note this implementation **does not** support SPI connected BMP280 devices.

The example code has provision to display temperature, pressure and altitude (adjusted for my current local QNH), but that code hardly fits inside an Attiny85. So please comment out the values you don't want to show.

The original code is released under a BSD license, and such is this. Please keep the Adafruit original notes in the code if you are going to adapt it, specially the note that follows:

## About this Driver ##

Adafruit invests time and resources providing this open source code.  Please support Adafruit and open-source hardware by purchasing products from Adafruit!

Written by Kevin (KTOWN) Townsend for Adafruit Industries.
