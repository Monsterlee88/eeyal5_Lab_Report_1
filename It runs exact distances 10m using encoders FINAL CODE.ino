//***********************************//
// ALEX LEE. EEYAL5                  //
// UNI OF NOTTINGHAM EEE             //
// 20459944                          //
//***********************************//

//CODE TO MAKE IT RUN FOR 10M LONG 
#include <Encoder.h>
#include <Servo.h>    //include the servo library
#define servoPin 4
Servo myservo;        // create servo object to control a servo
float steeringAngle;  // variable to store the servo position

#define enA 5   //EnableA command line - should be a PWM pin
#define enB 6   //EnableB command line - should be a PWM pin

//name the motor control pins - replace the CHANGEME with your pin number, digital pins do not need the 'D' prefix whereas analogue pins need the 'A' prefix
#define INa  A0 //PIN 19  //Channel A direction 
#define INb  A1 //PIN 20  //Channel A direction 
#define INc  A2 // PIN 21  //Channel B direction 
#define INd  A3 // PIN 22  //Channel B direction 

byte speedSetting = 0;  //initial speed = 0
Encoder myEnc(3, 12); //enable pins with interrupt capability
long oldPosition  = -999;
float distance;

void setup() 
{
  myservo.attach(servoPin);  //attach our servo object to pin D4
  //the Servo library takes care of defining the PinMode declaration (libraries/Servo/src/avr/Servo.cpp line 240)
  //configure the motor control pins as outputs
  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);   

  //initialise serial communication 
  Serial.begin(9600);
  speedSetting = 250; //speed of motors 
  Serial.print("Motor Speeds: ");
  Serial.println(speedSetting); 
}

void loop() 
{
  do
{
  myservo.write(90);
  goForwards();
  long newPosition = myEnc.read();
  // check if encoder has moved
  if (newPosition != oldPosition) 
  {
    oldPosition = newPosition;
    // output distance to the serial monitor   
    distance = newPosition * 0.7853981634;                     
    Serial.print("distance: ");
    Serial.println(distance);
  }
}
  while (distance < 1000);
  stopMotors();
  exit(1);
}

//for each of the below function, two of the 'IN' variables must be HIGH, and two LOW in order to move the wheels - use a trial and error approach to determine the correct combination for your EEEBot

void goForwards() 
{
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

void stopMotors() 
{
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}