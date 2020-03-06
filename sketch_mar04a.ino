#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
unsigned long time;
unsigned long startTime;
int LOOP_TIME = 50;
int MetersPerMillisecond = 10;
double xPos = 0;
double yPos = 0;
double zPos = 0;
void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}
 
void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  // time before count loop starts
  startTime = millis();
  

  //while (millis() < startTime + LOOP_TIME) {
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  xPos += (acc.x()) * 10;



  yPos += (acc.y()) * 10;




  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(xPos);
  Serial.print(" Y: ");
  Serial.print(yPos);
  //Serial.print(" Z: ");
 // Serial.print(acc.z());
  Serial.println("");
  //}
}
