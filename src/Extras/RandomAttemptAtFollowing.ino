// Use with 2 blue motors. This code does not have encoder code.

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define ENCA_1 D2
#define ENCA_2 D3

#define ENCB_1 D4
#define ENCB_2 D5

// Motor A (front left)
int enA = D12;
int in1 = D11;
int in2 = D10;

// Motor B (front right)
int enB = D7;
int in3 = D9;
int in4 = D8;

const float wheeldiameter = 66.10;
const float stepcount = 135.00;
volatile int ENCCount1 = 0;
volatile int ENCCount2 = 0;

void readEncoder1()
{
  int b1 = digitalRead(ENCA_1);
  ENCCount1++;
}

void readEncoder2()
{
  int b2 = digitalRead(ENCB_1);
  ENCCount2++;
}

int CMtoSteps(float cm)
{
  int result;                                        // Final calculation result
  float circumference = (wheeldiameter * 3.14) / 10; // Calculate wheel circumference in cm
  float cm_step = circumference / stepcount;         // CM per Step

  float f_result = cm / cm_step; // Calculate result as a float
  result = (int)f_result;        // Convert to an integer (note this is NOT rounded)

  return result; // End and return result
}

/* 
runMotors()
  Runs the motors at specified speed and for a specified length.
  Inputs: int steps, int speed
*/
void runMotors(int step, int speed)
{
  while (step > ENCCount1 && step > ENCCount2)
  {
    if (step > ENCCount1)
    {
      analogWrite(enA, speed);
    }
    else
    {
      analogWrite(enA, 0);
    }
    if (step > ENCCount2)
    {
      analogWrite(enB, speed);
    }
    else
    {
      analogWrite(enB, 0);
    }
  }
  // Stop when done
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  // Reset all counters
  ENCCount1 = 0; //  reset counter A to zero
  ENCCount2 = 0; //  reset counter B to zero
}

// Function to Move Forward
void MoveForward(int steps, int speed)
{
  // Set Motor A forward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // Set Motor B forward
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Run the motors at the specified speed, and amount of time
  runMotors(steps, mspeed);
}

// Function to Move Reverse
void MoveReverse(int steps, int speed)
{
  // Set Motor A reverse
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Set Motor B reverse
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Run the motors at the specified speed, and amount of time
  runMotors(steps, mspeed);
}

// Function to Spin Right
void SpinRight(int steps, int speed)
{
  // Set Motor A reverse
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // Set Motor B forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Run the motors at the specified speed, and amount of time
  runMotors(mspeed, mspeed);
}

// Function to Spin Left
void SpinLeft(int steps, int speed)
{
  // Set Motor A forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Set Motor B reverse
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Run the motors at the specified speed, and amount of time
  runMotors(mspeed, mspeed);
}

void setup()
{
  Serial.begin(9600);
  // On an STM32, setting the pinmode is necessary it seems. In an arduino, it automatically does it.
  pinMode(D12, OUTPUT);
  pinMode(D11, OUTPUT);
  pinMode(D10, OUTPUT);
  pinMode(D9, OUTPUT);
  pinMode(D8, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(ENCA_1, INPUT);
  pinMode(ENCA_2, INPUT);
  pinMode(ENCB_1, INPUT);
  pinMode(ENCB_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_2), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB_2), readEncoder2, RISING);
  // ****Motor Movement****
  /*
    255 is the max speed a motor can handle
    Blue motors can run at max 9V. 
    Battery pack I am using is a 8 pack AA pack, meaning it gives 12V (1.5V x 8), 
    Motor driver takes 1.4V about, meaning 10.6V is left for the motors.
    So 9/10.6 = x/255, therefore x = 215
    Therefore speeds that these blue motors can run at: Min = 140, Max = 215, (140-215)
  */
  // Format: MotorDirection(speed of motors (0-255), how ms long that action should last)

  MoveForward(140, 4000); // Ex: Forward at 140 speed for 4000 ms
  delay(1000);            // Wait one second
  MoveReverse(190, 4000);
  delay(1000); // Wait one second
  SpinRight(140, 4000);
  delay(1000); // Wait one second
  SpinLeft(200, 4000);
  delay(1000); // Wait one second
  MoveForward(190, 2000);
}

void loop()
{
}
