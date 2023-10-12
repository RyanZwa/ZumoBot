#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;

#define NUM_SENSORS 3
uint16_t lineSensorValues[NUM_SENSORS];


void setup() {
  Serial.begin(115200);
  lineSensors.initThreeSensors();
}

void loop() {
  //motors.setSpeeds(100, 100);
  //digitalWrite(11, LOW);
  lineSensors.read(lineSensorValues);
  
  Serial.print("0 : ");
  Serial.print(lineSensorValues[0]);
  Serial.print(" 1 : ");
  Serial.print(lineSensorValues[1]);
  Serial.print(" 2 : ");
  Serial.println(lineSensorValues[2]);
}
