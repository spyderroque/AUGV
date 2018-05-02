#include <string.h>
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <SPI.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"

#include "DualVNH5019Mod.h"

/*=========================================================================
    APPLICATION SETTINGS BLUETOOTH

      FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
     
                                Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                                running this at least once is a good idea.
     
                                When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                                Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
         
                                Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/
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
/*=========================================================================*/
#define wheelradius 60 //60mm acc. Pololu and another website wheeldiameter is 120 mm
/*=========================================================================*/

/*==================================IMU BNO055 Section=====================*/
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();
/*=========================================================================*/

/* Create the bluefruit object, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/**************************************************************************/
/*
   Functons and variables for the bluetooth LE module
*/
/**************************************************************************/
// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];
char ProgState = 'O'; //ProgState contains the state of Driving mode: 'O' Off, 'D' Linear Drive, 'R' Rotation, 'C' Circle Drive

uint16_t ADistance;
float ASpeed;
uint16_t AAngle;
double ARotSpeed;
float ARadius;
uint16_t AInterval;
unsigned long ATimer;
uint16_t RotIntSection;
imu::Vector<3> euler;

uint8_t PWMleft; //Speed signal on the PWM pin for left side
uint8_t PWMright; //Speed signal on the PWM pin for right side
uint8_t PWMstart[2];      //Initial Speed signal first value is left, seconed value is right
double phi_dot[2]; // collects the controller given value for Ticks/s
uint32_t LinTicks[2]; //collects the converted value to ticks a motor has to make for a defined distance

//volatile variables for the quadrature encoders
volatile int16_t LeftTicks;
volatile int16_t RightTicks;
volatile int16_t TicksCounterLeft = 0;
volatile int16_t TicksCounterRight = 0;

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)

            Also initiates the motor driver
*/
/**************************************************************************/
void setup(void) {
/*==============================Activate Serial and BLE===================*/
  // Serial.begin(115200);

  /*
     As before BLE first
  */
  // Serial.println(F("CameraStage Code Loaded to Controller"));
  // Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  // Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  // Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    // Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  // Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  // Serial.println(F("Please use RC3Rover app to connect"));
  // Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  // Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    // Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  // Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  // Serial.println(F("******************************"));

  /*==============Activate IMU===============================================*/
  // ble.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    // ble.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  // ble.print("Current Temperature: ");
  // ble.print(temp);
  // ble.println(" C");
  // ble.println("");

  bno.setExtCrystalUse(true);

  // ble.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  /*===========================Interrup=====================================*/
  attachInterrupt(leftencoder, countleft, CHANGE);
  attachInterrupt(rightencoder, countright, CHANGE);

  /*====================Motor driver========================================*/
 
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

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void) {
    // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  //imu::Vector<3> 
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT); //Timeout is in BluefruitConfig.h

  // Read command from Rc2Rover LE App
  // Command from App: Linear drive
  //Structure of Signal: !D1000LS5.1 1000 = 1000cm, 5.1 =5.1 km/h
  if (packetbuffer[1] == 'D') {
    char c[4];
    for (int i = 2; i < 6; i++) { //read given driving distance
      c[i - 2] = packetbuffer[i];
    }
    ADistance = atoi(c); //convert UTF-8 Ascii to integer
    for (int i = 8; i < 11; i++) { // read given cruising speed
      c[i - 8] = packetbuffer[i];
    }
    c[3] = '0';
    ASpeed = atof(c); //convert UTF-8 Ascii to float

    CalcLinDist(ADistance);
    SetLinVelocity(ASpeed);
    PWMleft = PWMstart[0];
    PWMright = PWMstart[1];

    ProgState = 'D';
  }
  
  if (packetbuffer[1] == 'O') {
    ProgState = 'O';
  }

  if (packetbuffer[1] == 'I') {
    if (packetbuffer[2] == 'R') {
      char c[3];
      char t[2];
      char numInterval[3];
      int timeconversionhelper;
      for (int i = 3; i < 6; i++) { //read given driving distance
        c[i - 3] = packetbuffer[i];
      }
      AAngle = atoi(c); //convert UTF-8 Ascii to integer

      t[0] = packetbuffer[8];//hours
      t[1] = packetbuffer[9];
      timeconversionhelper = atoi(t);
      ATimer = timeconversionhelper * 3600 * 1000; //conversion of hours to milliseconds

      t[0] = packetbuffer[11];//minutes
      t[1] = packetbuffer[12];
      timeconversionhelper = atoi(t);
      ATimer = ATimer + timeconversionhelper * 60 * 1000; // add minutes in milliseconds to the hours

      t[0] = packetbuffer[14];//hours
      t[1] = packetbuffer[15];
      timeconversionhelper = atoi(t);
      ATimer = ATimer + timeconversionhelper * 1000; // add seconds in milliseconds to the hours and minutes

      for (int i = 17; i < 20; i++) { // read given cruising speed
        numInterval[i - 17] = packetbuffer[i];
      }

      AInterval = atoi(numInterval); //convert UTF-8 Ascii to float

      ble.print(AAngle);

      ASpeed = 2.0;

      CalcRotIntAngle(AAngle, AInterval); //BERECHNET LINTICKS! DAS BRAUCHE ICH NICHT MEHR MIT IMU!!! Jetzt rechnet die Funktion Rotationssegmente
      SetLinVelocity(ASpeed); //no mistake. The SetLinVelocity function is used intentionally
      PWMleft = PWMstart[0]; //PWMstart WIRD DURCH FUNKTIONSAUFRUFE AUS SetRotVelocity erzeugt
      PWMright = PWMstart[1];

      ProgState = '3';
      ble.print ("AAngle: "); ble.println (AAngle);
      ble.print ("ATimer: "); ble.println (ATimer);
      ble.print ("AInterval "); ble.println (AInterval);
      ble.print ("ASpeed: "); ble.println (ASpeed);
      ble.print ("Programmstatus: "); ble.println (ProgState);

    }
    else if (packetbuffer[2] == 'C') {

    }
  }


  switch (ProgState) {
    case 'D':
      DriveLinear(PWMleft, PWMright, &LinTicks[0]);
      break;
    case '3':
      IntervalRotation(PWMleft, PWMright, &LinTicks[0]);
      break;
    case 'O':

      break;
  }
  MotorControl (&PWMleft, &PWMright, &LeftTicks, &RightTicks);


  LeftTicks = RightTicks = 0;

}



/**************************************************************************/
/*
   Interrupt Functions for the quadrature Encoders
*/
/**************************************************************************/

void countleft() {
  LeftTicks++;
  TicksCounterLeft++;
}

void countright() {
  RightTicks++;
  TicksCounterRight++;
}

/**************************************************************************/
/*
   Functions and variables for the motor driver
*/
/**************************************************************************/
void stopIfFault()
{
  if (getM1Fault()) {
    ble.println("M1 fault");
    while (1);
  }
  if (getM2Fault())  {
    ble.println("M2 fault");
    while (1);
  }
}

unsigned char getM1Fault() {
  return !digitalRead(EN1DIAG1);
}
unsigned char getM2Fault() {
  return !digitalRead(EN2DIAG2);
}

/******************************************************************************
   The following functions will drive the Rover according to the app instruction
*******************************************************************************/
void DriveLinear(uint8_t PWMleft, uint8_t PWMright, uint32_t LinTicks[0]) {
  if (TicksCounterLeft < LinTicks[0] + 1) {

    analogWrite(PWM1, PWMleft);
    digitalWrite(INA1, HIGH);
    digitalWrite(INB1, LOW);

    analogWrite(PWM2, PWMright);
    digitalWrite(INA2, HIGH);
    digitalWrite(INB2, LOW);

    stopIfFault();
    delay(20);
  }
  else {
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, LOW);
    analogWrite(PWM1, 127);

    digitalWrite(INA2, LOW);
    digitalWrite(INB2, LOW);
    analogWrite(PWM2, 127);

    stopIfFault();
    delay(20);
    TicksCounterLeft = 0;
    ProgState = 'O';
  }
}


void IntervalRotation(uint8_t PWMleft, uint8_t PWMright, uint32_t LinTicks[0]) {

  static int Intervalcounter;
  static unsigned long Timecounter;
  static unsigned long TargetAngle;
  static unsigned long OldAngle;
  unsigned long TimePresent;
  static bool MustSwitch = false;


  if (millis() > Timecounter + ATimer && Intervalcounter < AInterval+1) {
	  
    if (Intervalcounter == 0) { // case for the first interval of a sequence Timecounter is 0 at this point and millis() > than ATimer
      //Serial.print ("Intervallcounter: "); Serial.println (Intervalcounter);
      Timecounter = millis();
	  Intervalcounter++;
	  TargetAngle = euler.x() + RotIntSection;
	  MustSwitch = false;
	  if (TargetAngle >=360) {
		  TargetAngle = TargetAngle-360;
	  }
	  if ( euler.x() >= TargetAngle){
		  MustSwitch = true;
	  }
	  // ble.print("TargetAngle: ");
	  // ble.println(TargetAngle);
	  // ble.print("MustSwitch: ");
	  // ble.println(MustSwitch);
	  }
	
	else if (euler.x() < TargetAngle) {
	  analogWrite(PWM1, PWMleft);
      digitalWrite(INA1, HIGH);
      digitalWrite(INB1, LOW);

      analogWrite(PWM2, PWMright);
      digitalWrite(INA2, LOW);
      digitalWrite(INB2, HIGH);

      stopIfFault();
      delay(20);
    }
	
   	else if (MustSwitch == true && euler.x() >= (360 + TargetAngle - RotIntSection)){ 
	/* A Different notation resulted in "else if" ignoring nested if loops to change "MustSwitch" condition to false. 
	No clue why it is that way. For this reason a 360° turn in one interval is a problem */
	  // ble.println("Marker1");
	  analogWrite(PWM1, PWMleft);
      digitalWrite(INA1, HIGH);
      digitalWrite(INB1, LOW);

      analogWrite(PWM2, PWMright);
      digitalWrite(INA2, LOW);
      digitalWrite(INB2, HIGH);

      stopIfFault();
      delay(20);
	  
	  //MustSwitch = false;
      //ble.println("Marker3, MustSwitch aus");
	  // ble.print("MustSwitch: ");
	  // ble.println(MustSwitch);
	  
	  // ble.println("Marker2");
	  
	  
	}
	
    else {
      digitalWrite(INA1, LOW);
      digitalWrite(INB1, LOW);
      analogWrite(PWM1, 127);

      digitalWrite(INA2, LOW);
      digitalWrite(INB2, LOW);
      analogWrite(PWM2, 127);

      stopIfFault();
      delay(20);

      Intervalcounter++;
      Timecounter = millis();
	  TargetAngle = TargetAngle + RotIntSection;
	  MustSwitch = false;
	  if (TargetAngle >=360) {
		  TargetAngle = TargetAngle-360;
	  }
	  if ( euler.x() >= TargetAngle){
		  MustSwitch = true;
	  }
      //Serial.print ("Intervallcounter: "); Serial.println (Intervalcounter);
    }
  }

  else if (millis() > Timecounter + ATimer && Intervalcounter == AInterval+1) {
    //Serial.print ("Intervallcounter im Endmodus: "); Serial.println (Intervalcounter);
    Intervalcounter = 0;    
    ProgState = 'O';
  }
  else {
    //ble.println("Error in IntervalRotation");
    //Serial.print("Timecounter + ATimer -millis(): "); Serial.println(Timecounter + ATimer - millis());
  }
    
}


/*******************************************************
   This function converts the velocity to PWM Signal
 ******************************************************/
void SetLinVelocity (float velocity) {
  /* returns the PWM Signal needed max. PWM Signal=255 min=0
      rem: w=v/r; phi'=1600*K-784*exp(-0.0135*PWM); f=phi'/272; w=2*M_PI*f; r=wheelradius
      phi' = ENCODER TICKS/sec(!!!), K = 1-0.49*exp(0.85), PWM = PWM Signal [0...255],
      ==> PWM = 1700/27-2000/27*ln((1600-phi')/784); phi' = 272/(2*M_PI)*v/r
      PWM         phi' [1/s]  v [km/h]
      20,03552934 200,4173357 1
      25,53860253 300,6260036 1,5
      31,48355453 400,8346715 2
      37,94759274 501,0433394 2,5
      45,03011356 601,2520072 3
      52,86211465 701,4606751 3,5
      61,62119641 801,669343  4
      71,55667876 901,8780109 4,5
      83,03431293 1002,086679 5
      96,62238405 1102,295347 5,5
      113,2758656 1202,504014 6
      134,7933296 1302,712682 6,5
      165,2448988 1402,92135  7

  */
  double PWM;
  for (int i = 0; i < 2; i++) {
    phi_dot[i] = 272 / (2 * M_PI) * (velocity * 1000) / (wheelradius * 3.6); // 3.6 factor km/h = 3.6 m/s; 1000 factor 1000:  mm --> m  mm --> m
    PWM = Convert_phi_dot2PWM (phi_dot[i]);
    PWMstart[i] = (int)PWM + 0.5;
  }
}


double Convert_phi_dot2PWM (double arg) {
  double PWM;
  PWM = 1700 / 27 - 2000 / 27 * log((1600 - arg) / 784);
  return PWM;
}


/******************************************************
   These functions calculate the number of encoder ticks in order to return the distance
 ******************************************************/

void CalcLinDist(uint16_t ADistance) {
  /* Distance l = PHI*r (wheelradius); # rotations c=PHI/(2*PI); 272 c = phi (rem phi = # encoder ticks)
      i.e. phi = 136*l/(pi*r)
  */
  LinTicks[0] = (136 / M_PI) * (ADistance * 10 / wheelradius); // 10 = factor cm to mm
}




void CalcRotIntAngle(uint16_t AAngle, uint16_t AInterval) {

  /*
     Determine the current angle and then calculate the target angle.
  */

/* OBSOLETE STUFF THANKS TO THE BNO055 :-)
  //float Helper = (136 * AAngle * sqrt(75 * 75 + 127.5 * 127.5)) / (AInterval * 180 * wheelradius * cos(atan(127.5 / 75))); //127.5mm are half of the wheel base 75mm is half of the wheel distance
  
  //float Helper = (136 * AAngle * sqrt(75 * 75 + 127.5 * 127.5)) / (AInterval * 180 * wheelradius * cos(75/sqrt(75 * 75 + 127.5 * 127.5))); //WIE OBIGE FORMEL ABER ATAN IST ERSETZT WORDEN. ABER WAHRSCHEINLICH FALSCH.

  //float Helper = (136 * AAngle * sqrt(75 * 75 + 127.5 * 127.5)) / (AInterval * 180 * wheelradius * (75/sqrt(75 * 75 + 127.5 * 127.5))); //127.5mm are half of the wheel base 75mm is half of the wheel distance


  LinTicks[0] = (int) Helper + 0.5;
  LinTicks[1] = (int) Helper + 0.5;
  
 */
 
 RotIntSection = AAngle/AInterval;
  
}



/*******************************************************
 * *****************************************************
   This is the implementation of a simple P-Controller in order to keep the Thumper on track
 * *****************************************************
 * *****************************************************/

void MotorControl (uint8_t *PWM_L, uint8_t *PWM_R, volatile int16_t *Ticks_L, volatile int16_t *Ticks_R) {

  float DeltaNL;
  float DeltaNR;
  float KrL;
  float KrR;

  KrL = 10.584 * exp(-0.0135 * (PWMstart[0]));
  DeltaNL = phi_dot[0] - *Ticks_L / 0.02; //0.02 as delay after Motorsignal is 20 (aka 20millisec)

  KrR = 10.584 * exp(-0.0135 * (PWMstart[1]));
  DeltaNR = phi_dot[1] - *Ticks_R / 0.02; //0.02 as delay after Motorsignal is 20 (aka 20millisec)

  *PWM_L = PWMstart[0] + ((int) DeltaNL / KrL + 0.5);
  *PWM_R = PWMstart[1] + ((int) DeltaNR / KrR + 0.5);
}
