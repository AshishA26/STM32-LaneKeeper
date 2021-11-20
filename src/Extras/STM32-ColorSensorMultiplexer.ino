#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Use with 4 color sensors. Works with STM32.

#define numberOfSensors 4 // Put the number of color sensors you have here!

Adafruit_TCS34725 tcs[] = {Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X)
                          };

void chooseBus(uint8_t bus) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << (bus)); // will be using 0-3 SDA/SCL pins
  Wire.endTransmission();
}

void setup() {
  Serial.begin(9600);
  //Serial.println("Color View Test!");
  Wire.begin(); //Remember this line, very important
  for (int i = 0; i < numberOfSensors; i++) {
    Serial.println(i);
    chooseBus(i);
    if (tcs[i].begin()) {
      Serial.print("Found sensor "); Serial.println(i + 1);
    } else {
      Serial.println("No Sensor Found");
      while (true);
    }
  }
  delay(2000);
  Serial.println("start");
}

void loop() {
  for (int i = 0; i < numberOfSensors; i++) {
    if (i==1 || i == 2){continue;} // This is for stopping it from reading data from color sensor 1 and 2, and only allows 0 and 3, comment out if wanted
    chooseBus(i);
    float red, green, blue;
    //tcs[i].setInterrupt(false);  // turn on LED
    delay(60);  // takes 50ms to read
    tcs[i].getRGB(&red, &green, &blue);
    //tcs[i].setInterrupt(true);  // turn off LED
    
    // Comment out this section below if wanting to use with colorviewCollectorPDE (available in Arduino Cookbook)!
    // Serial.print("Sensor: ");
    // Serial.print(i);
    // Serial.print("\t");
    
    Serial.print("R:\t"); Serial.print(int(red));
    Serial.print("\tG:\t"); Serial.print(int(green));
    Serial.print("\tB:\t"); Serial.print(int(blue));
    Serial.print("\n");
  }
}
