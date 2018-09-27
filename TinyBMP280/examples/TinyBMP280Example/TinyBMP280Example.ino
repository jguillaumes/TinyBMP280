#include <TinyBMP280.h>
#include <SoftwareSerial.h>

//+
// Test sketch for the TinyBMP280 library.
// For use with an Attiny85:
// - Connect BMP280 SCL to pin 7
// - Connect BMP280 SDA to pin 8
// - Connect BMP280 SDO to ground
// - Connect a serial terminal using:
//   - TX to Pin 2 (corresponds to Arduino pin 3)
//   - RX to pin 3 (corresponds to Arduino pin 4)
// - Setup your terminal or serial monitor to 2400 bauds
// - Enjoy!

SoftwareSerial mySerial(3, 4);
tbmp280::TinyBMP280 bmp;

void setup() {
	  mySerial.begin(2400);
	  mySerial.println("BEGIN:");
	  bmp.begin();
}

void loop() {

	float t = bmp.readTemperature();
	float p = bmp.readPressure();
	float h = bmp.readAltitude(1027.0); // Replace this value for your current QNH

	mySerial.print("T: "); mySerial.println(t);
	mySerial.print("P: "); mySerial.println(p/100);
	mySerial.print("H: "); mySerial.println(h);
	mySerial.println("");

	delay(1000);

}
