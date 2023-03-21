/*
 * Contains all code necessary to control and setup head servos
 */
#include <Arduino.h>
#include <Servo.h> // servo library

/**
 * Runs head-specific setup code.
 */
void setupHead();

/**
 * Moves head servos to target positions over a specified number of steps/iterations.
 * @param z The z position of the head platform (not used by ROS)
 * @param roll The roll of the head platform (0 - 180 deg)
 * @param pitch The pitch of the head platform (0 - 180 deg)
 * @param yaw The roll of the yaw platform (0 - 180 deg)
 * @param LEY The left ear yaw (0 - 180 deg)
 * @param LEP The left ear pitch (0 - 180 deg)
 * @param REY The right ear yaw (0 - 180 deg)
 * @param REP The right ear pitch (0 - 180 deg)
 * @param iters The number of iterations it should take to reach target positions.
 */
void runHead(double z, double roll, double pitch, double yaw, int LEY, int LEP, int REY, int REP, double iters);

/**
 * Not used outside of head file
 */
void unitCubicCreate(double tf);

/**
 * Not used outside of head file
 */
void angleCalculate(int arr[3], double z, double roll, double pitch, double yaw);

/**
 * loop() implementation for testing Head servos
 */
void testHeadLoop();
