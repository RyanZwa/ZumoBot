#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4LineSensors lineSensors;

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

void turnRight() {
  motors.setSpeeds(turnSpeed, -turnSpeed);
}

void turnLeft() {
  motors.setSpeeds(-turnSpeed, turnSpeed);
}

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
  delay(400);
}

int accDeceleration(bool objectSeen) {
  if (objectSeen) {
    // An object is visible, so we will start decelerating in
    // order to help the robot find the object without
    // overshooting or oscillating.
    turnSpeed -= deceleration;
  } else {
    // An object is not visible, so we will accelerate in order
    // to help find the object sooner.
    turnSpeed += acceleration;
  }

  // Constrain the turn speed so it is between turnSpeedMin and
  // turnSpeedMax.
  turnSpeed = constrain(turnSpeed, turnSpeedMin, turnSpeedMax);

  return turnSpeed;
}

void follow(bool objectSeen, uint8_t leftValue, uint8_t rightValue) {

  if (objectSeen) {

    if (leftValue < rightValue) {
      // The right value is greater, so the object is probably
      // closer to the robot's right LEDs, which means the robot
      // is not facing it directly.  Turn to the right to try to
      // make it more even.
      turnRight();
    } else if (leftValue > rightValue) {
      // The left value is greater, so turn to the left.
      turnLeft();
    } else {
      motors.setSpeeds(maxSpeed, maxSpeed);
    }
  }
}
void dodgeTheLine() {
  if (lineSensorValues[0] < lineSensorValue || lineSensorValues[1] < lineSensorValue || lineSensorValues[2] < lineSensorValue) {
    //Serial.print("WIT : ");
    turnAround();
  }
}

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

void setup() {
  Serial.begin(115200);
  proxSensors.initThreeSensors();
  lineSensors.initThreeSensors();
  randomSeed(analogRead(0));
  startTurn();
}

void loop() {

  lineSensors.read(lineSensorValues);
  //dodgeTheLine();
  proxSensors.read();
  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
  uint8_t rightValue = proxSensors.countsFrontWithRightLeds();

  bool objectSeen;
  objectSeen = (leftValue >= sensorThreshold) || (rightValue >= sensorThreshold);

  turnSpeed = accDeceleration(objectSeen);

  if (leftValue == 0 && rightValue == 0) {
    motors.setSpeeds(maxSpeed, maxSpeed);
  }

  follow(objectSeen, leftValue, rightValue);
}
