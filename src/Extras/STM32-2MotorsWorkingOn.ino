#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <TimerOne.h>

const byte MOTOR_A = 3; // Motor 1 Interrupt Pin - INT 1 - Front Left Motor
const byte MOTOR_B = 2; // Motor 2 Interrupt Pin - INT 0 - Front Right Motor

const float stepcount = 20.00; // 20 Slots in disk, change if different
const float wheeldiameter = 66.10; // Wheel diameter in millimeters, change if different

// Integers for pulse counters
volatile int counter_A = 0;
volatile int counter_B = 0;

// Motor A (front right)
int enA = D12;
int in1 = D11;
int in2 = D10;

// Motor B (front left)
int enB = D7;
int in3 = D9;
int in4 = D8;

// ****Interrupt Service Routines****
// Motor A pulse count ISR
void ISR_countA()
{
  counter_A++; // increment Motor A counter value
}

// Motor B pulse count ISR
void ISR_countB()
{
  counter_B++; // increment Motor B counter value
}

// Function to convert from centimeters to steps
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
resetCounters()
  Resets all step counters to 0.
  Inputs: None
*/
void resetCounters()
{
  counter_A = 0; //  reset counter A to zero
  counter_B = 0; //  reset counter B to zero
}

/* 
runMotors()
  Runs the motors at specified speed and for a specified length.
  Inputs: int steps, int speed
*/
void runMotors(int step, int speed)
{
  while (step > counter_A && step > counter_B)
  {
    if (step > counter_A)
    {
      analogWrite(enA, speed);
    }
    else
    {
      analogWrite(enA, 0);
    }
    if (step > counter_B)
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
  resetCounters();
}

// Function to Move Forward
void MoveForward(int steps, int mspeed)
{
  // Reset all counters
  resetCounters();

  // Set Motor A forward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // Set Motor B forward
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Run motors for specified steps and speed
  runMotors(steps, mspeed);
}

// Function to Move Reverse
void MoveReverse(int steps, int mspeed)
{
  // Reset all counters
  resetCounters();

  // Set Motor A reverse
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Set Motor B reverse
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Run motors for specified steps and speed
  runMotors(steps, mspeed);
}

// Function to Spin Right
void SpinRight(int steps, int mspeed)
{
  // Reset all counters
  resetCounters();

  // Set Motor A reverse
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // Set Motor B forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Run motors for specified steps and speed
  runMotors(steps, mspeed);
}

// Function to Spin Left
void SpinLeft(int steps, int mspeed)
{
  // Reset all counters
  resetCounters();

  // Set Motor A forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Set Motor B reverse
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Run motors for specified steps and speed
  runMotors(steps, mspeed);
}

void setup()
{
  // On an STM32, setting the pinmode is necessary it seems. In an arduino, it automatically does it.
  pinMode(D12, OUTPUT);
  pinMode(D11, OUTPUT);
  pinMode(D10, OUTPUT);
  pinMode(D9, OUTPUT);
  pinMode(D8, OUTPUT);
  pinMode(D7, OUTPUT);

  // Attach the Interrupts to their ISR's
  attachInterrupt(digitalPinToInterrupt(MOTOR_A), ISR_countA, RISING); // Increase counter A when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt(MOTOR_B), ISR_countB, RISING); // Increase counter B when speed sensor pin goes High

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

  // Test Motor Movement:

  MoveForward(CMtoSteps(20), 215); // Ex: Forward at 255 speed for 20cm
  delay(1000);                     // Wait one second
  MoveReverse(CMtoSteps(20), 195);
  delay(1000); // Wait one second
  SpinRight(CMtoSteps(20), 160);
  delay(1000); // Wait one second
  SpinLeft(CMtoSteps(20), 200);
  delay(1000); // Wait one second
  MoveForward(CMtoSteps(20), 215);
}

void loop()
{
}
