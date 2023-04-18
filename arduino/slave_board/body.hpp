#include <Arduino.h>
#include <Servo.h> // servo library

#define RIGHT_ARM_SHOULDER_PIN 3
#define RIGHT_ARM_ROTATOR_PIN 5
#define RIGHT_ARM_ELBOW_PIN 6
#define LEFT_ARM_SHOULDER_PIN 9
#define LEFT_ARM_ROTATOR_PIN 10
#define LEFT_ARM_ELBOW_PIN 11

void setupBody();

void runBody(int chassis, int las, int lar, int lae, int ras, int rar, int rae);
