#include <Arduino.h>
#include <Servo.h> // servo library

#define CHASSIS_PIN 10
#define RIGHT_ARM_SHOULDER_PIN 11
#define RIGHT_ARM_ROTATOR_PIN 12
#define RIGHT_ARM_ELBOW_PIN 13
#define LEFT_ARM_SHOULDER_PIN 14
#define LEFT_ARM_ROTATOR_PIN 15
#define LEFT_ARM_ELBOW_PIN 16

void setupBody();

void runBody(int chassis, int las, int lar, int lae, int ras, int rar, int rae);
