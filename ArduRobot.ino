#include <Servo.h>

Servo gEyes;
const int KServoPin = 9;
const int KTrigPin = 2;
const int KEchoPin = 3;

void setup() {
  // init motor

  // init ultrasound wave
  pinMode(KTrigPin,OUTPUT);
  pinMode(KEchoPin, INPUT);
  
  // init servo
  gEyes.attach(KServoPin);
  gEyes.write(15);
  gEyes.delay(300);
  gEyes.write(0);
  gEyes.delay(300);
}

void loop() {
  // put your main code here, to run repeatedly:

}

// double motor move in sync
void arMove()
{
  
}

// double motor stop in sync
void arStop()
{
  
}

// stop left motor, while right motor continued
//   moving distance by angle
void arTurnLeftBy(int aAngle)
{
  
}

// stop left motor, while right motor continued
//   moving distance by angle
void arTurnLeftTo(int aAngle)
{
  
}

// turn servo, thus can make ultrasound wave unit 
//    to detect new distance
void arTurnFindPath()
{
  
}

// return distance with ultrasound wave unit
long arDistance()
{
  long distance;
  // trigger ultrasound wave
  digitalWrite(KTrigPin, LOW);
  delayMicroseconds(2); 
  digitalWrite(KTrigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(KTrigPin, LOW); 

  distance = pulseIn(KEchoPin, HIGH) / 58.00;
  return distance;
}

// return delay time, calculated by angle and speed
//  trigonometric function
long arMoveDistance(int angle)
{
  
}

