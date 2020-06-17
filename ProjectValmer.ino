//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library

#include "RoboClaw.h"
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
void setupIMU();

void setMotorPowerTank(double leftPower, double rightPower);
const int ROBOCLAW_MAX_POWER = 128;
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler();
RoboClaw roboclaw(&Serial1,10000);
RoboClaw roboclaw2(&Serial2,10000);


enum robotState {
  STARTUP,
  MOVING,
  STOPPED,
  FIRING
};

unsigned long long TIME_SINCE_STOP = 0;
unsigned long DELAY_BETWEEN_STOPS = 10000;
unsigned long STOP_TIME = 1000;
#define roboClawSerialAddress 0x80


double MOTOR_SPEED_MAX = 1; 
double ROBOT_HEADING = 0;

double TARGET_HEADING = 90;
double HEADING_ERROR = TARGET_HEADING - ROBOT_HEADING;
double LEFT_POWER = 0;
double RIGHT_POWER = 0;
double Kp = 0.01;
// FOR BANG BANG CONTROL BECAUSE PID IS FOR PROPERLY MADE THINGS
double CORRECTION_THRESH = 25;


bool MOVEMENT_DETECTED = false;
double STOP_TIMESTAMP = millis();

robotState ROBOT_STATE = STARTUP;
void setup() {
  Serial.begin(9600);
  setupIMU();
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

  switch(ROBOT_STATE) {
    
    case STARTUP:
      ROBOT_STATE = MOVING;
      TIME_SINCE_STOP = millis();
      break;
    case MOVING:
      STOP_TIMESTAMP = millis();
      while (millis() < STOP_TIMESTAMP + STOP_TIME) {
        ROBOT_HEADING = getRobotHeading();
        Serial.print(ROBOT_HEADING);
        Serial.print("      ");
        HEADING_ERROR = TARGET_HEADING - ROBOT_HEADING;
        if (HEADING_ERROR > CORRECTION_THRESH) {
          LEFT_POWER = 0.5;
          RIGHT_POWER = 0.25;
        } else if (HEADING_ERROR < -CORRECTION_THRESH) {
          LEFT_POWER = 0.25;
          RIGHT_POWER = 0.5;
        } else {
          LEFT_POWER = 0.5;
          RIGHT_POWER = 0.5;
        }
        Serial.print(LEFT_POWER);
        Serial.print("      ");
        Serial.println(RIGHT_POWER);
        setMotorPowerTank(LEFT_POWER,RIGHT_POWER);
      }
      ROBOT_STATE = STOPPED;
      break;
    case STOPPED:
      setMotorPowerTank(0,0);
      STOP_TIMESTAMP = millis();
      while(millis() < STOP_TIMESTAMP + STOP_TIME) {
        // use ulrasonic sensor to detect movement 
        if (MOVEMENT_DETECTED) {
          ROBOT_STATE = FIRING;
          break;
        } else {
          ROBOT_STATE = MOVING;
        }
      }
      
      break;
    case FIRING:
      break;
  }


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

/*
 * Function sets up IMU 
 */
void setupIMU() {
 /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
}

double getRobotHeading() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  return euler.x();
}


