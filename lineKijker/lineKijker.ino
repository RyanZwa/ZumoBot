/*
this program two ways for following a line 
if you want to use one way you can call the ProportionalController()
or the OnOffController() funtion in the main loop. NEVER call both at the same time 
*/
#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonB buttonB;

#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];
int16_t lastError = 0;
const uint16_t maxSpeed = 200;
int buttonState = 0;

void CalibrateSensors() {
  delay(1000);
  for (uint16_t i = 0; i < 120; i++) {
    if (i > 30 && i <= 90) {
      motors.setSpeeds(-200, 200);
    } else {
      motors.setSpeeds(200, -200);
    }
    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

void OnOffController(int16_t position) {
  int16_t error = position - 2000;
  if (position > 2000) {
    motors.setSpeeds(maxSpeed, 0);
  }
  if (position < 2000) {
    motors.setSpeeds(0, maxSpeed);
  }
}

void ProportionalController(int16_t position) {
  if (position > 2000) {
    int16_t corr = map(position, 0, 2000, maxSpeed, 0);
    motors.setSpeeds(maxSpeed - corr, 0);
  }
  if (position < 2000) {
    int16_t corr = map(position, 2000, 4000, 0, maxSpeed);

    motors.setSpeeds(0, maxSpeed - corr);
  }
}

void Buttons() {
  if (buttonState == 0) {
    buttonB.waitForButton();
    CalibrateSensors();
    buttonB.waitForButton();
    buttonState++;
  }
}
void setup() {
  lineSensors.initFiveSensors();
}

void loop() {
  int16_t position = lineSensors.readLine(lineSensorValues);

  Buttons();
  ProportionalController(position);
}
