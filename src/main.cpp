#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
#include "color_constants.h"
#include "PID.h"

#define RED 1
#define GREEN 2
#define BLUE 3
#define OTHER 0

#define LEFT 20
#define RIGHT 21
#define NONE 22
#define STOP 23

#define carSpeed 165

#define numberOfSensors 4 // Put the number of color sensors you have here!

Adafruit_TCS34725 tcs[] = {Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X)};

// Motor A (front left)
int ENA = D12;
int IN1 = D11;
int IN2 = D10;

// Motor B (front right)
int ENB = D7;
int IN3 = D9;
int IN4 = D8;

// Pins
#define NMOTORS 2
#define M0 0
#define M1 1
const int enca[] = {D2, D5};
const int encb[] = {D3, D4};
const int pwm[] = {D12, D7};
const int in1[] = {D11, D9};
const int in2[] = {D10, D8};

// Euclidian Distance for colors
float RedDist = 0;
float GreenDist = 0;
float BlueDist = 0;
int ColorFound_R = 0;
int ColorFound_L = 0;

// globals
long prevT = 0;         // previous tim
int posPrev[] = {0, 0}; // previous encoder counts
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i[] = {0, 0}; // encoder count accumulated in interrupt

float vel_rpm_filtered[] = {0, 0};
float vel_rpm_prev[] = {0, 0};

SimplePID pid[NMOTORS];
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
  analogWrite(pwm, pwmVal); // Motor speed
  if (dir == 1)
  {
    // Turn one way
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1)
  {
    // Turn the other way
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else
  {
    // Or dont turn
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

template <int j>
void readEncoder()
{
  int b = digitalRead(encb[j]);
  if (b > 0)
  {
    pos_i[j]++;
  }
  else
  {
    pos_i[j]--;
  }
}

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

    if (i == 0)
    {
      if (findColor(red, green, blue) == RED)
      {
        return RIGHT;
      }
      if (findColor(red, green, blue) == GREEN)
      {
        return STOP;
      }
    }
    if (i == 3)
    {
      if (findColor(red, green, blue) == RED)
      {
        return LEFT;
      }
      if (findColor(red, green, blue) == GREEN)
      {
        return STOP;
      }
    }
  }
  return NONE;
}

float vt[NMOTORS] = {0, 0};
int SPEED = 15;
void forward()
{
  vt[0] = -SPEED;
  vt[1] = -SPEED;
  // analogWrite(ENA, carSpeed);
  // analogWrite(ENB, carSpeed);
  // digitalWrite(IN1, LOW);
  // digitalWrite(IN2, HIGH);
  // digitalWrite(IN3, LOW);
  // digitalWrite(IN4, HIGH);
  // Serial.println("Forward");
}

void backword()
{
  vt[0] = SPEED;
  vt[1] = SPEED;
  // analogWrite(ENA, carSpeed);
  // analogWrite(ENB, carSpeed);
  // digitalWrite(IN1, HIGH);
  // digitalWrite(IN2, LOW);
  // digitalWrite(IN3, HIGH);
  // digitalWrite(IN4, LOW);
  // Serial.println("Back");
}

void left()
{

  vt[0] = SPEED;
  vt[1] = -SPEED;
  // analogWrite(ENA, carSpeed);
  // analogWrite(ENB, carSpeed);
  // digitalWrite(IN1, LOW);
  // digitalWrite(IN2, HIGH);
  // digitalWrite(IN3, HIGH);
  // digitalWrite(IN4, LOW);
  // Serial.println("Left");
}

void right()
{
  vt[0] = -SPEED;
  vt[1] = SPEED;
  // analogWrite(ENA, carSpeed);
  // analogWrite(ENB, carSpeed);
  // digitalWrite(IN1, HIGH);
  // digitalWrite(IN2, LOW);
  // digitalWrite(IN3, LOW);
  // digitalWrite(IN4, HIGH);
  // Serial.println("Right");
}

void stop()
{
  SPEED = 0;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  pinMode(D12, OUTPUT);
  pinMode(D11, OUTPUT);
  pinMode(D10, OUTPUT);
  pinMode(D9, OUTPUT);
  pinMode(D8, OUTPUT);
  pinMode(D7, OUTPUT);

  for (int k = 0; k < NMOTORS; k++)
  {
    pid[k].setParams(50, 3, 0, 215);
  }
  attachInterrupt(digitalPinToInterrupt(enca[M0]), readEncoder<M0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M1]), readEncoder<M1>, RISING);

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
  delay(500);
}

void update_motors()
{

  // read the position
  long pos[2];    // temp variable, reading current pos from interrupt var
  noInterrupts(); // disable interrupts temporarily while reading
  for (int k = 0; k < NMOTORS; k++)
  {
    pos[k] = pos_i[k];
  }
  interrupts(); // turn interrupts back on

  // Compute velocity
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;

  for (int k = 0; k < NMOTORS; k++)
  {
    float vel_temp_ticks = (pos[k] - posPrev[k]) / deltaT; // calculating current ticks/s
    posPrev[k] = pos[k];
    // Convert count/s to RPM
    float vel_rpm = vel_temp_ticks / 600.0 * 60.0; // rpm. temp

    // Low-pass filter (25 Hz cutoff)
    // vel_rpm_filtered = 0.854 * vel_rpm_filtered + 0.0728 * vel_rpm + 0.0728 * vel_rpm_prev; // Original 25Hz cutoff
    vel_rpm_filtered[k] = 0.96907 * vel_rpm_filtered[k] + 0.015465 * vel_rpm + 0.015465 * vel_rpm_prev[k]; // Our 5Hz cutoff
    vel_rpm_prev[k] = vel_rpm;
  }

  // Loop through the motors
  for (int k = 0; k < NMOTORS; k++)
  {
    int pwr, dir;                                               // calculated by PID
    pid[k].evalu(vel_rpm_filtered[k], vt[k], deltaT, pwr, dir); // compute the position
    setMotor(dir, pwr, pwm[k], in1[k], in2[k]);                 // signal the motor
  }
  for (int i = 0; i < 2; i++)
  {
    Serial.print(-vt[i]);
    Serial.print(" ");
    Serial.print(-vel_rpm_filtered[i]);
    Serial.print(" ");
  }
  Serial.println();
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
  case STOP:
    stop();
    break;
  default:
    forward();
    break;
  }
  update_motors();
  delay(2); // 2.4ms integration time
}
