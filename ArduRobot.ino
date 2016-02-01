#include <Servo.h>

const int KServoPin = 9;
const int KTrigPin = 2;
const int KEchoPin = 3;
const int KLeftMotorIn1Pin = 4;
const int KLeftMotorIn2Pin = 5;
const int KRightMotorIn1Pin = 6;
const int KRightMotorIn2Pin = 7;

const int KSpeed = 40; // centimeter/second
const int KCollisionDistance = 20; // centimeter
const int KMaxLeftAngle = 160;
const int KMaxRightAngle = 20;
const float KPi = 3.14;

Servo gEyes;

// 1 running
// 0 stopped and checking path
// 2 finding path complete
int gRun = 1;

// previous distance to measure
int gPrevDistance = KCollisionDistance; 

int gServoPos = 90;

// 30, 60, 90, 120, 150
int gArrDistance[5] = {0};

int gNoiseCount = 0;

void setup() {
  // init motor
  pinMode(KLeftMotorIn1Pin, OUTPUT);
  pinMode(KLeftMotorIn2Pin, OUTPUT);
  pinMode(KRightMotorIn1Pin, OUTPUT);
  pinMode(KRightMotorIn2Pin, OUTPUT);
  
  // init ultrasound wave
  pinMode(KTrigPin,OUTPUT);
  pinMode(KEchoPin, INPUT);
  
  // init servo
  gEyes.attach(KServoPin);
  gEyes.write(KMaxRightAngle);
  delay(500);
  gEyes.write(KMaxLeftAngle);
  delay(500);
  gEyes.write(gServoPos);

  Serial.begin(9600);
}

void loop() {

  if (gRun == 0) {
    // stopped and finding path
    int findingDistance = arDistance();

    if (arNoise(findingDistance)) {
      // increase noise count
      ++gNoiseCount;
    } else {
      if (gServoPos < 150) {
        // fill finding distance array
        // step1: fill the distance array
        if (gServoPos == 30) {
          gArrDistance[0] = findingDistance;
        } else if (gServoPos == 60) {
          gArrDistance[1] = findingDistance;
        } else if (gServoPos == 90) {
          gArrDistance[2] = findingDistance;
        } else if (gServoPos == 120) {
          gArrDistance[3] = findingDistance;
        } 

        // step2: calculating new postion (+30 degree)
        gServoPos += 1;

        // step4: move servo to new postion
        gEyes.write(gServoPos);

        delay(10);
      } else {
        // fill finding distance array is over
        // step1: fill the last element, 150 degree
        gArrDistance[4] = findingDistance;

        // step2: reset
        gServoPos = 90;

        // step3: set status to "found path"
        gRun = 2;

        // step4: reset the servo position
        gEyes.write(gServoPos);
      }
    }
  } else if (gRun == 2) {
    // finding path is over
    // step1: find max distance
    int maxId = 0;
    int maxDistance = gArrDistance[0];
    boolean bFoundMax = false;
    
    for (int i = 1; i < 4; ++i) {
      if ( gArrDistance[i] > maxDistance) {
        Serial.println(i);
        maxId = i;
        maxDistance = gArrDistance[i];
        bFoundMax = true;
      }
    }

    if (!bFoundMax) {
      arBackward();
      delay(500);
    } else{
      // if 90 degree has max distance, backward
      if (maxId == 2) {
        arBackward();
      } else if (maxId < 2) {
        arBackward();

        arStop();
        // should turn right
        arTurnRightTo(90-(maxId+1)*30);

        delay(300);
      } else {
        arBackward();

        arStop();
        // should turn left
        arTurnLeftTo((maxId-2)*30);
        delay(300);
      }
    }

    gRun = 1;
  } else { // gRun == 1
    // running
    int distance = arDistance();

    if (arNoise(distance)) {
      // noise, ignore
      return;
    }
    
    if (distance < KCollisionDistance) {
      // step 1: stop
      arStop();
  
      // step2: set status to not running, thus can make servo to find path 
      gRun = 0;
  
      // step3: remember current distance
      gPrevDistance = distance;
  
      // step4: servo to 30 degrees
      gServoPos = 30;
      gEyes.write(gServoPos);
      
    } else {
      gRun = 1;
      arForward();
    }
  }
} 

//void loop() {
// gEyes.write(gServoPos);
//}

// double motor move in sync
void arForward()
{
  // left motor forward
  digitalWrite(KLeftMotorIn2Pin, LOW);
  digitalWrite(KLeftMotorIn1Pin, HIGH);

  // right motor forward
  digitalWrite(KRightMotorIn2Pin, LOW);
  digitalWrite(KRightMotorIn1Pin, HIGH);
}

// double motor stop in sync
void arStop()
{
    // left motor stopped
  digitalWrite(KLeftMotorIn2Pin, LOW);
  digitalWrite(KLeftMotorIn1Pin, LOW);

  // right motor stopped
  digitalWrite(KRightMotorIn2Pin, LOW);
  digitalWrite(KRightMotorIn1Pin, LOW);
}

void arBackward()
{
    // left motor forward
  digitalWrite(KLeftMotorIn2Pin, HIGH);
  digitalWrite(KLeftMotorIn1Pin, LOW);

  // right motor forward
  digitalWrite(KRightMotorIn2Pin, HIGH);
  digitalWrite(KRightMotorIn1Pin, LOW);

  delay(1000);
}
// stop left motor, while right motor continued
void arTurnLeftTo(int aAngle)
{
  long delayTime = arMoveDistance(aAngle);
      // left motor stop
  digitalWrite(KLeftMotorIn2Pin, LOW);
  digitalWrite(KLeftMotorIn1Pin, LOW);

  // right motor forward
  digitalWrite(KRightMotorIn2Pin, LOW);
  digitalWrite(KRightMotorIn1Pin, HIGH);

Serial.println("turn left");
Serial.println(delayTime);

  // delay
  delay(delayTime);
}

// stop right motor, while left motor continued
void arTurnRightTo(int aAngle)
{
    long delayTime = arMoveDistance(aAngle);
      // left motor forward
  digitalWrite(KLeftMotorIn2Pin, LOW);
  digitalWrite(KLeftMotorIn1Pin, HIGH);

  // right motor stop
  digitalWrite(KRightMotorIn2Pin, LOW);
  digitalWrite(KRightMotorIn1Pin, LOW);

Serial.println("turn right");
Serial.println(delayTime);
  // delay
  delay(delayTime);
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
  float dis = ((float)angle) / 360 *2*KPi*KCollisionDistance;
Serial.println(dis);
  return (long) (dis/KSpeed *1000);
}

boolean arNoise(int distance)
{
  // judging noise or not, change infactor to improve accuracy
  if (distance >= gPrevDistance*50) {
    return true;
  } else {
    return false;
  }
}

