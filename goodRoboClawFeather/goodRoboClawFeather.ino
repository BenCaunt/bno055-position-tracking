//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library

#include "RoboClaw.h"
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function


/*
 * Function Prototypes 
 */
void setMotorPowerTank(double leftPower, double rightPower);

/*
 * Constants
 */
const int ROBOCLAW_MAX_POWER = 128;

/*
 * Adapt Digital PINS 11 and 10 to a UART Serial connection usinng SERCOM
 */
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler();

/*
 * Create Roboclaw Objects 
 */
RoboClaw roboclaw(&Serial1,10000);
RoboClaw roboclaw2(&Serial2,10000);

/*
 * Serial Address of Arduino
 */
#define roboClawSerialAddress 0x80


void setup() {
  //Open roboclaw serial ports
  roboclaw.begin(38400);
  roboclaw2.begin(38400);
  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  // small delay in setup so that the back right motor doesnt start randomly spinning
  // TODO: find elegant solution to this lol. 
  setMotorPowerTank(0,0);
  
}

void loop() {

  setMotorPowerTank(0.5,0.5);
  delay(2000);
  setMotorPowerTank(0,0);
  delay(2000);
  setMotorPowerTank(-0.5,0.5);
  delay(2000);
  setMotorPowerTank(0,0);
  delay(2000);

}


/*
 * Set power as if robot is a tank drive
 * 
 * ++ -> drive straight forward
 * -- -> drive straight backward
 * +- -> differential turn right
 * -+ => differential turn left
 *      _______
 *    O |     | O
 *      |     |
 * L    |     |   R
 *    O |_____| O
 *   
 * 
 */
void setMotorPowerTank(double leftPower, double rightPower) {

  // make copies of motor powers that are properly scaled out between 128 and 0 
  int scaledLeftPower = convertMotorPower(leftPower);
  int scaledRightPower = convertMotorPower(rightPower);

  // use these methods if left side drives forward, if not use the second set in else 
  if (leftPower < 0) {
    roboclaw.ForwardM1(roboClawSerialAddress,scaledLeftPower); 
    roboclaw.BackwardM2(roboClawSerialAddress,scaledLeftPower); 
  } else {
    roboclaw.BackwardM1(roboClawSerialAddress,scaledLeftPower);
    roboclaw.ForwardM2(roboClawSerialAddress,scaledLeftPower);
  }

  if (rightPower < 0) {
    roboclaw2.BackwardM1(roboClawSerialAddress,scaledRightPower); 
    roboclaw2.ForwardM2(roboClawSerialAddress,scaledRightPower); 
  } else {
    roboclaw2.ForwardM1(roboClawSerialAddress,scaledRightPower);
    roboclaw2.BackwardM2(roboClawSerialAddress,scaledRightPower);
  }

}

/*
 * Function converts 1 to -1 power level into 0 to 128
 * Function assumes that wherever it is called is changing the proper roboclaw method.  
 * 
 */
double convertMotorPower(double power) {
  power = abs(power);
  // ensure power is between 0 and 1 because I am very paranoid about where this code may be used. 
  if (power > 1) {
    power = 1;
  } else if (power < 0) {
    power = 0;
  }
  return power * ROBOCLAW_MAX_POWER;
}

// function to handle setup of our SERCOM to UART conversion
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}
