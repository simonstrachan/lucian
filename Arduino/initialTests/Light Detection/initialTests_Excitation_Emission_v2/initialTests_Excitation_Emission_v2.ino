#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "Adafruit_TCS34725.h"

#define ledPIN 6
#define ledCount 

uint16_t r, g, b, c, colorTemp, lux;

// Initialise with specific int time and gain values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

Adafruit_NeoPixel pixels(ledCount, ledPIN, NEO_RGB + NEO_KHZ800);


void setup(void) {
  excitation();
  Serial.begin(9600);

  //Check connection
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

}

void loop(void) {
  emissionReading();
  serialPrint();
}

void excitation(){
  strip.begin();
  strip.show();
  for(int i; i < ledCount; i++){
    pixels.setPixelColor(i, pixels.Color(0, 0, 255));
  }
}

void emissionReading(){
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);
}

void serialPrint(){
  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");
}

/*  Links 
 * TCS34725 git: https://github.com/adafruit/Adafruit_TCS34725
 * NeoPixel project: https://create.arduino.cc/projecthub/electropeak/neopixel-how-to-control-ws2812-rgb-led-w-arduino-46c08f
 */
