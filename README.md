# TinyBMP280: a driver for BMP280/BMP180 sensor devices under AVR tiny MCUs

This is a version of the ADAFRUIT_BMP280_Library library tailored to run
on AVR tiny devices. It replaces the standard Arduino I2C library by the
TinyWireM one, which uses an USI-based implementation of the I2C protocol. The library also supports the BMP180 device, using the same contract as the BMP280. The BMP180 support is based on the Adafruit' BMP280 library with some bits copied from the BMP085 (unified sensors) one.

This version of the library can replace the Adafruit one as a drop-in. The public interface is compatible so very few (if any) code modification will be needed. Please take note this implementation **does not** support SPI connected BMP280 devices.

The example code has provision to display temperature, pressure and altitude (adjusted for my current local QNH), but that code hardly fits inside an Attiny85. So please comment out the values you don't want to show.

The original code is released under a BSD license, and such is this. Please keep the Adafruit original notes in the code if you are going to adapt it, specially the note that follows:

## Usage

### Warning: this library uses a explicit C++ namespace

This libary uses *namespaces*. The namespace it uses is ```tbmp```. That's not (unfortunately) a general practice in Arduino libraries, but in my opinion the advantages they provide outweight the *very small* inconvenients.

Which are those inconvenients? Well, that's easy to answer. In the general case, the *inconvenient* is you will have to add this line at the top of your sketch (below the #include statements):

```
using namespace tbmp;
```

In most cases, that will be it. That tells the C++ compiler that should try to find unqualified class names in the specified namespace. That will work **unless** there is a name clash with another library who happens to use the TinyBMPBase, TinyBMP280 or TinyBMP180 names (as well as some other internal ones). In that case, what you must do is to qualify the class names with the ```tbmp``` qualifier. That is, instead of declaring your sensor as:

```
TinyBMP280 myBmp;
```
You must qualify the class name:
```
tbmp::TinyBMP280 myBmp;
```

 And that's it. The example sketch uses this second method.

### Library reference

This library defines two public use classes, with the same general contract:

- **TinyBMP180** will drive a BMP180 device connected via I2C.
- **TinyBMP280** will drive a BMP280 device connected via I2C.

The BMP280 supports both I2C and SPI connection. However, this library does only implement I2C.

The public methods (functions) are:

#### Common methods

These methods apply to both classes.

- ```TinyBMP180()``` / ```TinyBMP280()```: Public constructors. Use them to declare your device.
- ```int32_t readIntTemperature()```: Gets a temperature reading in centigrades multiplied by 100. That is, a value of 1234 means 12.34 C. Notice the value is **signed**.
- ```uint32_t readIntPressure()```: Gets a pressure reading in Pascals. You must divide it by 100 to get the more commonly used hectoPascals (hPa). A value of 101234 means 1012.34 hPa. Notice the value is **unsigned**.
- ```float readTemperature()```: Gets a temperature reading in centigrades as a float value. No further conversion required.
- ```float readPressure()```: Gets a pressure reading in hectopascals as a float value. No further conversion needed.
- ```float readAltitude(float qnh)```: Gets an altutude reading in meters as a float value. The input parameter is the so called qnh (in aviation). That is the theoretical or measured current atmospheric pressure at sea level for the point where the measurement is taken. You can get this value looking at the weather broadcast for your nearest airport. Those values are part of the METAR data which is commonly published in the internet. For example, here:

https://www.aviationweather.gov/metar

You have to provide the 4-letter ICAO code for the airport and choose the "decoded format". For instance, the value I get while writing this for the LEBL (Barcelona) airport is:

```Pressure (altimeter):	30.06 inches Hg (1018.0 mb)```

The value I should provide to readAltitude() is 1018.0.

#### Methods specific of TinyBMP180

- ```void begin(bmp180_mode_t mode)```: Intializes the device in the specified mode. The default is ``BMP180_MODE_ULTRAHIGHRES```. Please refer to the BMP180 datasheet for details about the working modes.

#### Methods specific of TinyBMP280

- ```void begin(uint8_t ic2Address)```: Initializes the device and sets its I2C address. The default is 0x76, which corresponds to a wiring of the SDO pin to GND. The 280 can be set up at 0x76 (with SDO wired to GND) or 0x77 (with SDO wired to 3.3V). The BMP180 address is fixed to 0x77.


## About this Driver ##

Adafruit invests time and resources providing this open source code.  Please support Adafruit and open-source hardware by purchasing products from Adafruit!

Written by Kevin (KTOWN) Townsend for Adafruit Industries.
