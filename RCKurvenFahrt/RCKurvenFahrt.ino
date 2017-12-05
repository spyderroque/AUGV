#include <string.h>
#include <Arduino.h>
#include <SPI.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
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

/* Create the bluefruit object, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* Create the DualVNH5019 motor driver object */
DualVNH5019MotorShield md;

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

uint16_t Speed; //Speed variable values range from -400 to +400


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)

            Also initiates the motor driver
*/
/**************************************************************************/
void setup(void)
{
//  while (!Serial);  // required for Flora & Micro
//  delay(500);

  Serial.begin(115200);

  /*
     As before BLE first
  */
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the game controller!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

  /*========================================================================*/
  /*
     Now the motor driver
  */
  md.init();
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);

  // Check
  if (packetbuffer[1] != 'B') {
    Serial.println("App is not in Controller-GUI! Please change interface.");
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';

    if (buttnum <= 4){
      Speed= SetVelocity(buttnum);
    }
    if (buttnum ==5){
      DriveForward(Speed, pressed);
    }
    if (buttnum ==8){
      TurnRight(Speed, pressed);
    }
    if (buttnum ==6){
      DriveBackward(Speed, pressed);
    }
    if (buttnum ==7){
      TurnLeft(Speed, pressed);
    }

    
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    }
    else {
      Serial.println(" released");
    }
  }

}

/**************************************************************************/
/*
   Functions and variables for the motor driver
*/
/**************************************************************************/
void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while (1);
  }
  if (md.getM2Fault())
  {
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
void DriveForward(uint8_t velocity, boolean Buttonpress) {
  if (Buttonpress) {
    for (int i = 0; i <= velocity; i++) {
      md.setM1Speed(i);
      md.setM2Speed(i);
      stopIfFault();
      if (i % 200 == 100) {
        Serial.print("M1 current: ");
        Serial.println(md.getM1CurrentMilliamps());
      }
      delay(2);
    }
  }
  else {
    md.setM1Speed(0);
    md.setM2Speed(0);
  }
}

void DriveBackward(uint8_t velocity, boolean Buttonpress) {
  if (Buttonpress) {
    for (int i = 0; i <= velocity; i++) {
      md.setM1Speed(-i);
      md.setM2Speed(-i);
      stopIfFault();
      if (i % 200 == 100) {
        Serial.print("M1 current: ");
        Serial.println(md.getM1CurrentMilliamps());
      }
      delay(2);
    }
  }
  else {
    md.setM1Speed(0);
    md.setM2Speed(0);
  }
}

void TurnLeft(uint8_t velocity, boolean Buttonpress) {
  if (Buttonpress) {
    for (int i = 0; i <= velocity; i++) {
      md.setM1Speed(i/2);
      md.setM2Speed(i);
      stopIfFault();
      if (i % 200 == 100) {
        Serial.print("M1 current: ");
        Serial.println(md.getM1CurrentMilliamps());
      }
      delay(2);
    }
  }
  else {
    md.setM1Speed(0);
    md.setM2Speed(0);
  }
}

void TurnRight(uint8_t velocity, boolean Buttonpress) {
  if (Buttonpress) {
    for (int i = 0; i <= velocity; i++) {
      md.setM1Speed(i);
      md.setM2Speed(i/2);
      stopIfFault();
      if (i % 200 == 100) {
        Serial.print("M1 current: ");
        Serial.println(md.getM1CurrentMilliamps());
      }
      delay(2);
    }
  }
  else {
    md.setM1Speed(0);
    md.setM2Speed(0);
  }
}

uint16_t SetVelocity (uint8_t ButtonNum) {
  /* 4 = Full Speed
     3 = Half Speed, i.e.   200/400 dutycycles
     2 = quarter Speed      100/400
     1 = 1/8 speed          50/400
  */
  if (ButtonNum == 1) {
    return 100;
  }
  if (ButtonNum == 2) {
    return 200;
  }
  if (ButtonNum == 3) {
    return 300;
  }
  if (ButtonNum == 4) {
    return 400;
  }
}

