#include <string.h>
#include <Arduino.h>
#include <SPI.h>


#include "DualVNH5019Mod.h"

#define  INA1     10
#define  INB1     11
#define  EN1DIAG1 13
#define  CS1      A0
#define  INA2      0
#define  INB2     19
#define  EN2DIAG2 1
#define  CS2      A1
#define  PWM1     5
#define  PWM2     6

#define leftencoder    16 //pin A2 Interrupt 9
#define rightencoder   17 //pin A3 Interrupt 4


uint16_t Speed; //Speed variable values range from -400 to +400

//volatile variables for the quadrature encoders
volatile int16_t LeftTicks;
volatile int16_t RightTicks;

/****************************************************************/
/****************************************************************/

void setup(void)
{
  Serial.begin(115200);

  /*========================================================================*/
  attachInterrupt(leftencoder, countleft, CHANGE);
  attachInterrupt(rightencoder, countright, CHANGE);

  /*========================================================================*/
  /*
     Now the motor driver
  */
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(EN1DIAG1, INPUT);
  pinMode(CS1, INPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(EN2DIAG2, INPUT);
  pinMode(CS2, INPUT);

  delay(2000);

  for (int i = 0; i < 256; i++) {
    DriveForward(i);
    Serial.print("PWM Sigmal: ");
    Serial.print(i);
    Serial.print(" Linker Encoder: ");
    Serial.print(LeftTicks  );
    Serial.print(" Rechter Encoder: ");
    Serial.println(RightTicks);
    LeftTicks = RightTicks = 0;
  }
  for (int i = 255; i > 0; i--) {
    DriveForward(i);
    Serial.print("PWM Sigmal: ");
    Serial.print(i);
    Serial.print(" Linker Encoder: ");
    Serial.print(LeftTicks  );
    Serial.print(" Rechter Encoder: ");
    Serial.println(RightTicks);
    LeftTicks = RightTicks = 0;
  } 
  DriveForward(0);
}

/**************************************************************************/
/*!
    @brief  Drive each motor side once PWM 0 to 255
*/
/**************************************************************************/
void loop(void) {


}
/**************************************************************************/
/*
   Interrupt Functions for the quadrature Encoders
*/
/**************************************************************************/

void countleft() {
  LeftTicks++;
}

void countright() {
  RightTicks++;
}

/**************************************************************************/
/*
   Functions and variables for the motor driver
*/
/**************************************************************************/
void stopIfFault()
{
  if (getM1Fault()) {
    Serial.println("M1 fault");
    while (1);
  }
  if (getM2Fault())  {
    Serial.println("M2 fault");
    while (1);
  }
}


/*
   The following functions will check what kin of button has been pressed.
   If it is button 1 to 4 it will set the PWM for motor speed. If it is buttons
   5 to 8 it will check if the button is pressed and drive the motors accordingly

   TO BE LATER REPLACED BY A DIRECT ACCESS OF THE MOTOR DRIVER, C.F. DUALVNH5019Mod.cpp
*/
void DriveForward(uint8_t velocity) {
  if (velocity != 0) {

    analogWrite(PWM1, velocity);
    digitalWrite(INA1, HIGH);
    digitalWrite(INB1, LOW);

    analogWrite(PWM2, velocity);
    digitalWrite(INA2, HIGH);
    digitalWrite(INB2, LOW);
    stopIfFault();
    delay(500);
  }
  else {
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, LOW);
    analogWrite(PWM1, 255);
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, LOW);
    analogWrite(PWM2, 255);
  }
}

void DriveBackward(uint8_t velocity, boolean Buttonpress) {
  if (Buttonpress) {
    analogWrite(PWM1, velocity * 51 / 80);
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, HIGH);

    analogWrite(PWM2, velocity * 51 / 80);
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, HIGH);
    stopIfFault();
    delay(2);
  }
  else {
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, LOW);
    analogWrite(PWM1, 400 * 51 / 80);
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, LOW);
    analogWrite(PWM2, 400 * 51 / 80);
  }
}

void TurnLeft(uint8_t velocity, boolean Buttonpress) {
  if (Buttonpress) {
    analogWrite(PWM1, velocity * 51 / 80);
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, HIGH);

    analogWrite(PWM2, velocity * 51 / 80);
    digitalWrite(INA2, HIGH);
    digitalWrite(INB2, LOW);
    stopIfFault();
    delay(2);
  }
  else {
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, LOW);
    analogWrite(PWM1, 400 * 51 / 80);
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, LOW);
    analogWrite(PWM2, 400 * 51 / 80);
  }
}

void TurnRight(uint8_t velocity, boolean Buttonpress) {
  if (Buttonpress) {
    analogWrite(PWM1, velocity * 51 / 80);
    digitalWrite(INA1, HIGH);
    digitalWrite(INB1, LOW);

    analogWrite(PWM2, velocity * 51 / 80);
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, HIGH);
    stopIfFault();
    delay(2);
  }
  else {
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, LOW);
    analogWrite(PWM1, 400 * 51 / 80);
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, LOW);
    analogWrite(PWM2, 400 * 51 / 80);
  }
}

unsigned char getM1Fault() {
  return !digitalRead(EN1DIAG1);
}

unsigned char getM2Fault() {
  return !digitalRead(EN2DIAG2);
}
