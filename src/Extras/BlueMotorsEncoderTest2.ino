#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
//#include <TimerOne.h>

#define ENCA D2
#define ENCB D3

void setup()
{
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
}

void loop()
{
  int a = digitalRead(ENCA);
  int b = digitalRead(ENCB);
  Serial.print(a * 5);
  Serial.print(" ");
  Serial.print(b * 5);
  Serial.println();
}