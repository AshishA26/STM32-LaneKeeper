#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_TCS34725.h>

// Define the experimental constants for the color sensor
#define COLOR_RED_R_EBAY 197.57
#define COLOR_RED_G_EBAY 42.86
#define COLOR_RED_B_EBAY 40.43
#define COLOR_RED_MAX_DISTANCE_EBAY 30.7

#define COLOR_GREEN_R_EBAY 106.41
#define COLOR_GREEN_G_EBAY 106.35
#define COLOR_GREEN_B_EBAY 50.35
#define COLOR_GREEN_MAX_DISTANCE_EBAY 21.4

#define COLOR_BLUE_R_EBAY 63.14
#define COLOR_BLUE_G_EBAY 98.29
#define COLOR_BLUE_B_EBAY 109
#define COLOR_BLUE_MAX_DISTANCE_EBAY 7.8

#define RED 1
#define GREEN 2
#define BLUE 3
#define OTHER 0

#define LEFT 20
#define RIGHT 21
#define NONE 22

#define carSpeed 200

#define numberOfSensors 4 // Put the number of color sensors you have here!

Adafruit_TCS34725 tcs[] = {Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X)};

//Line Tracking IO define
int colorSensor_R;
int colorSensor_L;

// Motor A (front left)
int ENA = D12;
int IN1 = D11;
int IN2 = D10;

// Motor B (front right)
int ENB = D7;
int IN3 = D9;
int IN4 = D8;

// Euclidian Distance for colors
float RedDist = 0;
float GreenDist = 0;
float BlueDist = 0;
int ColorFound_R = 0;
int ColorFound_L = 0;

void chooseBus(uint8_t bus)
{
  Wire.beginTransmission(0x70);
  Wire.write(1 << (bus)); // will be using 2-3 instead of 0-1 SDA/SCL pins because of convience (placed better on the breadboard)
  Wire.endTransmission();
}

float findColor(float R_New, float G_New, float B_New)
{
  RedDist = sqrt(pow(R_New - COLOR_RED_R_EBAY, 2) + pow(G_New - COLOR_RED_G_EBAY, 2) + pow(B_New - COLOR_RED_B_EBAY, 2));
  GreenDist = sqrt(pow(R_New - COLOR_GREEN_R_EBAY, 2) + pow(G_New - COLOR_GREEN_G_EBAY, 2) + pow(B_New - COLOR_GREEN_B_EBAY, 2));
  BlueDist = sqrt(pow(R_New - COLOR_BLUE_R_EBAY, 2) + pow(G_New - COLOR_BLUE_G_EBAY, 2) + pow(B_New - COLOR_BLUE_B_EBAY, 2));
  if (RedDist < COLOR_RED_MAX_DISTANCE_EBAY)
  {
    return RED;
  }
  else if (GreenDist < COLOR_GREEN_MAX_DISTANCE_EBAY)
  {
    return GREEN;
  }
  else if (BlueDist < COLOR_BLUE_MAX_DISTANCE_EBAY)
  {
    return BLUE;
  }
  else
  {
    return OTHER;
  }
}

int readColor()
{
  for (int i = 0; i < numberOfSensors; i++)
  {
    // This is for stopping it from reading data from color sensor 1 and 2, and only allows 0 and 3
    if (i == 1 || i == 2)
    {
      continue;
    }

    chooseBus(i);
    float red, green, blue;
    tcs[i].getRGB(&red, &green, &blue);

    // Serial.print("Sensor: ");
    // Serial.print(i);
    // Serial.print("\t");
    // Serial.print("R:\t");
    // Serial.print(int(red));
    // Serial.print("\tG:\t");
    // Serial.print(int(green));
    // Serial.print("\tB:\t");
    // Serial.print(int(blue));
    // Serial.print("\n");

    if (i == 0 && findColor(red, green, blue) == RED)
    {
      return LEFT;
      // Serial.println("LEFT = red");
    }
    if (i == 3 && findColor(red, green, blue) == RED)
    {
      return RIGHT;
      // Serial.println("RIGHT = red");
    }
  }
  return NONE;
}

void forward()
{
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Forward");
}

void backword()
{
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Back");
}

void left()
{
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Left");
}

void right()
{
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Right");
}

void stop()
{
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  //Serial.println("Stop!");
}

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  pinMode(D12, OUTPUT);
  pinMode(D11, OUTPUT);
  pinMode(D10, OUTPUT);
  pinMode(D9, OUTPUT);
  pinMode(D8, OUTPUT);
  pinMode(D7, OUTPUT);

  for (int i = 0; i < numberOfSensors; i++)
  {
    Serial.println(i);
    chooseBus(i);
    if (tcs[i].begin())
    {
      Serial.print("Found sensor ");
      Serial.println(i);
    }
    else
    {
      Serial.println("No Sensor Found");
      while (true)
        ;
    }
  }
}

void loop()
{
  switch (readColor())
  {
  case LEFT:
    left();
    break;
  case RIGHT:
    right();
    break;
  default:
    forward();
    break;
  }
  delay(50); // takes 50ms to read
}
