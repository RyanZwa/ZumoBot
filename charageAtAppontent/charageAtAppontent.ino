#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;

const uint8_t sensorThreshold = 1;

const uint16_t turnSpeedMax = 400;
const uint16_t turnSpeedMin = 100;
const uint16_t deceleration = 10;
const uint16_t acceleration = 10;

// If the robot is turning, this is the speed it will use.
uint16_t turnSpeed = turnSpeedMax;



void turnRight() {
  motors.setSpeeds(turnSpeed, -turnSpeed);
}

void turnLeft() {
  motors.setSpeeds(-turnSpeed, turnSpeed);
}

void startTurn() {
  int left = proxSensors.countsLeftWithLeftLeds();
  int right = proxSensors.countsRightWithRightLeds();

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

void setup() {
  proxSensors.initFrontSensor();
  Serial.begin(115200);
  delay(10);
  Serial.println("kaasje");

  startTurn();
}

void loop() {

  // Read the front proximity sensor and gets its left value (the
  // amount of reflectance detected while using the left LEDs)
  // and right value.
  proxSensors.read();
  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
  uint8_t rightValue = proxSensors.countsFrontWithRightLeds();

  // Determine if an object is visible or not.
  bool objectSeen = (leftValue >= sensorThreshold) || (rightValue >= sensorThreshold);

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

  if (leftValue == 0 && rightValue == 0) {
    motors.setSpeeds(400, 400);
  }

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
      motors.setSpeeds(400, 400);
    }
  }
/*
  Serial.print("front");
  Serial.print(leftValue);
  Serial.print(", ");
  Serial.println(rightValue);
*/
}
