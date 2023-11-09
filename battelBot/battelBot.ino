#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4OLED display;

const uint8_t sensorThreshold = 1;
const uint16_t maxSpeed = 400;
const uint16_t turnSpeedMax = 400;
const uint16_t turnSpeedMin = 100;
const uint16_t deceleration = 10;
const uint16_t acceleration = 10;
const uint16_t lineSensorValue = 500;
const uint16_t straightSpeed = 200;
const uint16_t reverseSpeed = 400;

#define NUM_SENSORS 3
uint16_t lineSensorValues[NUM_SENSORS];


// If the robot is turning, this is the speed it will use.
uint16_t turnSpeed = turnSpeedMax;

//this function is for turning the robot right or left.
void turnRight() {
  motors.setSpeeds(turnSpeed, -turnSpeed);
}

void turnLeft() {
  motors.setSpeeds(-turnSpeed, turnSpeed);
}

//this function is for detecting and turning towards the enemy robot at the start of the game.
void startTurn() {
  proxSensors.read();
  uint8_t left = proxSensors.countsLeftWithLeftLeds();
  uint8_t right = proxSensors.countsRightWithRightLeds();

  Serial.print("langs");
  Serial.print(left);
  Serial.print(" , ");
  Serial.println(right);

  if (left < right) {
    turnRight();
  } else if (left > right) {
    turnLeft();
  }
  delay(200);
}
//this function decreases the speed when the enemy is in sight and increases when it isn't to find it faster.
int accDeceleration(bool objectSeen) {
  if (objectSeen) {
    turnSpeed -= deceleration;
  } else {
    turnSpeed += acceleration;
  }

  // Constrain the turn speed so it is between the minimal and maximal speed.
  turnSpeed = constrain(turnSpeed, turnSpeedMin, turnSpeedMax);

  return turnSpeed;
}

// This function checks where the enemy is located and responds to that.
void follow(bool objectSeen, uint8_t leftValue, uint8_t rightValue) {

  if (objectSeen) {
    //When the right value is greater, the object is closer to the right LEDs so in respond the robot will try to turn to face the enemy directly.
    if (leftValue < rightValue) {
      turnRight();
    }
    //When the left value is greater, the object is closer to the left LEDs so in respond the robot will try to turn to face the enemy directly.
    else if (leftValue > rightValue) {
      turnLeft();
    }
    //When the robot is facing the enemy directly it will go at it at full speed.
    else {
      motors.setSpeeds(maxSpeed, maxSpeed);
    }
  }
}
//When a sensor passes the line the robot will turn around.
void dodgeTheLine() {
  if (lineSensorValues[0] < lineSensorValue || lineSensorValues[1] < lineSensorValue || lineSensorValues[2] < lineSensorValue) {
    //turnAround();
    motors.setSpeeds(-maxSpeed, maxSpeed);
    delay(100);

  }
}
/*
//When turning away from a line the robot will go backwards, then turn either left or right and then go forward again.
void turnAround() {
  int randomDirection = random(2);
  motors.setSpeeds(-reverseSpeed, -reverseSpeed);
  delay(200);

  if (randomDirection == 0) {
    motors.setSpeeds(reverseSpeed, -reverseSpeed);
  } else {
    motors.setSpeeds(-reverseSpeed, reverseSpeed);
  }
  delay(200);
}
*/

void waitTime() {
  display.clear();
  display.print("Press A");
  buttonA.waitForButton();
  delay(5000);
  display.clear();
  display.gotoXY(0, 0);
  display.print("coole");
  display.gotoXY(0, 1);
  display.print("kikkers");
}
void setup() {
  Serial.begin(115200);
  proxSensors.initThreeSensors();
  lineSensors.initThreeSensors();
  //randomSeed(analogRead(0));

  waitTime();
  startTurn();
}

void loop() {

  lineSensors.read(lineSensorValues);
  dodgeTheLine();
  proxSensors.read();
  //leftValue and rightValue reads the most frontleft and most frontright sensors.
  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
  uint8_t rightValue = proxSensors.countsFrontWithRightLeds();
  //Checks if the robot sees the enemy or not.
  bool objectSeen;
  objectSeen = (leftValue >= sensorThreshold) || (rightValue >= sensorThreshold);

  turnSpeed = accDeceleration(objectSeen);
  //If leftValue and rightValue is 0, the robot will go max speed which will either make it smash into the enemy or go to a line.
  if (leftValue == 0 && rightValue == 0) {
    motors.setSpeeds(maxSpeed, maxSpeed);
  }
  //Follows the enemy.
  follow(objectSeen, leftValue, rightValue);
}