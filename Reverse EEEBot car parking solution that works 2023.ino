//***********************************//
// ALEX LEE. EEYAL5                  //
// UNI OF NOTTINGHAM EEE             //
// 20459944                          //
//***********************************//

// AUTONOMOUS CAR PARKING SOLUTION METHOD A

#include <Wire.h>
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal

//ULTRA SENSOR
#include <NewPing.h>
#define TRIGGER_PIN  26  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     34  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 

//IMU GYROSCOPE
#include <MPU6050_tockn.h>
MPU6050 mpu6050(Wire);

void setup()
{
  Serial.begin(9600);
  Wire.begin();   // join i2c bus (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)
  mpu6050.calcGyroOffsets(true);
}

void moveCar(int leftMotor_speed, int rightMotor_speed, int servoAngle) //A FUNCTION CREATED TO MOVE THE CAR THAN HAVING TO KEEP COPYING PASTING THIS CODE.
{
  Wire.beginTransmission(I2C_SLAVE_ADDR); 
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));  
  Wire.write((byte)(leftMotor_speed & 0x000000FF));         
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));    
  Wire.write((byte)(rightMotor_speed & 0x000000FF)); 
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));    
  Wire.write((byte)(servoAngle & 0x000000FF));  
  Wire.endTransmission();  
  delay(1000);
}

void getIMUdata(void)
{
  mpu6050.update();
  {
  Serial.println("=======================================================");
    Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
    Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
    Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
  
    Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
    Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
    Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
    
    Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
    Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
    Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
  Serial.println("=======================================================\n");
  }
}

void loop() //WHERE THE CODE BEGINS
{

moveCar(100, 100, 90); //MAKES THE CAR DRIVE FORWARDS (STRAIGHT) / HAS PAREMTERS WHICH WILL SET THE LEFT AND RIGHT MOTOR SPEED AND THE ANGLE(DIRECTION OF THE SERVO)
delay(1000); //X SECOND FOWARDS
moveCar(0, 0, 90); //STOP CAR

mpu6050.update();
double endangle = (mpu6050.getAngleZ() + 185);
do
{
getIMUdata();
moveCar(112, 112, 45); //rotate ccw 180 deg
}
while(mpu6050.getAngleZ() < endangle);

do
{
moveCar(-110, -110, 90);
Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
Serial.println("cm");
}
while(sonar.ping_cm() > 22 );

mpu6050.update();
double endangle2 = (mpu6050.getAngleZ() - 130);

do
{
moveCar(105, 105, 135); //rotate cw 90 deg
mpu6050.update();
}
while(mpu6050.getAngleZ() > endangle2 );

do
{
Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
Serial.println("cm");
moveCar(-120, -120, 90);
}
while(sonar.ping_cm() > 22 );
 
moveCar(0, 0, 90);
delay(2000);
}