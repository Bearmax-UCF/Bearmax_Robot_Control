/*
 * Contains all code necessary to control and setup head servos
 */
#include "head.hpp"

//Create objects
//no num is front, 2 is left, 3 is right
Servo frontPlatformServo; //front
Servo leftPlatformServo; //left
Servo rightPlatformServo; //right
Servo yawPlatformServo; //yaw servo
Servo leftEarYawServo; //left ear yaw
Servo leftEarPitchServo; //left ear pitch
Servo rightEarYawServo; //right ear yaw
Servo rightEarPitchServo; //right ear pitch

int angles[3]; //0-front, 1-left, 2-right
int servoAdjust[8]; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
int prevAngles[8];
double currentYaw = 0;
double prevPlatformPos[4]; //0-z, 1-roll, 2-pitch, 4-yaw
double servoDelay;
int iterations;
double unitCubicCoeff[2];

void setupHead(){
  frontPlatformServo.attach(2); //front
  leftPlatformServo.attach(3); //left
  rightPlatformServo.attach(4); //right
  yawPlatformServo.attach(5); //
  leftEarYawServo.attach(6); //
  leftEarPitchServo.attach(7); //
  rightEarYawServo.attach(8); //
  rightEarPitchServo.attach(9); //

  servoAdjust[0] = -5; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  servoAdjust[1] = -3; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  servoAdjust[2] = -1; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  servoAdjust[3] = -4; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  servoAdjust[4] = 3; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  servoAdjust[5] = -3; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  servoAdjust[6] = 0; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  servoAdjust[7] = -5; //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch

  frontPlatformServo.write(90 + servoAdjust[0]); //front
  leftPlatformServo.write(90 + servoAdjust[1]); //left
  rightPlatformServo.write(90 + servoAdjust[2]); //right
  yawPlatformServo.write(90 + servoAdjust[3]); 
  leftEarYawServo.write(90 + servoAdjust[4]); 
  leftEarPitchServo.write(90 + servoAdjust[5]); 
  rightEarYawServo.write(90 + servoAdjust[6]); 
  rightEarPitchServo.write(90 + servoAdjust[7]); 
  
  delay(1000);

  prevAngles[0] = prevAngles[1] = prevAngles[2] = prevAngles[3] = prevAngles[4] = prevAngles[5] = prevAngles[6] = prevAngles[7] = 90;
  //0-front, 1-left, 2-right, 3-patform yaw, 4-left ear yaw, 5-left ear pitch, 6-right ear yaw, 7-right ear pitch
  
  prevPlatformPos[0] = prevPlatformPos[1] = prevPlatformPos[2] = prevPlatformPos[3] = 0;
  
  delay(1000);
}

void runHead(double z, double roll, double pitch, double yaw, int LEY, int LEP, int REY, int REP, double iters){
  double ratio;
  double angleDiff[8];
  double positions[8];
  double unitCubicValue;
  positions[0] = z;
  positions[1] = roll;
  positions[2] = pitch;
  positions[3] = yaw;
  positions[4] = LEY;
  positions[5] = LEP;
  positions[6] = REY;
  positions[7] = REP;
  
  unitCubicCreate(iters);

  for(int i = 0; i <= 3; i++){
    angleDiff[i] = positions[i] - prevPlatformPos[i];
  }
  
  for(int i = 4; i <= 7; i++){
    angleDiff[i] = positions[i] - prevAngles[i];
  }

  if(abs(yaw - currentYaw) > 0){ //enter into yaw based control
    
    for(int i = 1; i <= iters; i++){
      ratio = (double)i / iters;

      unitCubicValue = unitCubicCoeff[0] * pow(i, 3) + unitCubicCoeff[1] * pow(i, 2);

      for(int i = 0; i <= 2; i++){ //positions of platform
        positions[i] = angleDiff[i] * unitCubicValue + prevPlatformPos[i];
      }
      
      currentYaw = unitCubicValue * angleDiff[3] + prevPlatformPos[3];
      
      for(int i = 4; i <= 7; i++){ //positions of ear servos
        positions[i] = angleDiff[i] * unitCubicValue + prevAngles[i];
      }
      
      angleCalculate(angles, positions[0], positions[1], positions[2], currentYaw); //z, roll, pitch, yaw

      frontPlatformServo.write(angles[0] + servoAdjust[0]); 
      leftPlatformServo.write(angles[1] + servoAdjust[1]); 
      rightPlatformServo.write(angles[2] + servoAdjust[2]); 
      yawPlatformServo.write(currentYaw + 90 + servoAdjust[3]); 
      leftEarYawServo.write(positions[4] + servoAdjust[4]); 
      leftEarPitchServo.write(positions[5] + servoAdjust[5]); 
      rightEarYawServo.write(positions[6] + servoAdjust[6]); 
      rightEarPitchServo.write(positions[7] + servoAdjust[7]);
      delay(servoDelay); 
    }

    for(int i = 0; i <= 2; i++){ //when done, make previous angles the end position of the motion
      prevAngles[i] = angles[i];
    }
    
    for(int i = 3; i <= 7; i++){ //when done, make previous angles the end position of the motion
      prevAngles[i] = positions[i];
    }
  }
  
  else{ //enter into no yaw basted control

  angleCalculate(angles, positions[0], positions[1], positions[2], positions[3]); //z, roll, pitch, yaw
  
    for(int i = 1; i <= iters; i++){
      ratio = (double)i / iters;

      unitCubicValue = unitCubicCoeff[0] * pow(i, 3) + unitCubicCoeff[1] * pow(i, 2);

      for(int i = 0; i <= 2; i++){ //positions of platform servos
        positions[i] = angles[i] * unitCubicValue + prevAngles[i] * (1 - unitCubicValue);
      }
      for(int i = 4; i <= 7; i++){ //positions of ear and yaw servos
        positions[i] = angleDiff[i] * unitCubicValue + prevAngles[i];
      }

      frontPlatformServo.write(positions[0] + servoAdjust[0]); 
      leftPlatformServo.write(positions[1] + servoAdjust[1]); 
      rightPlatformServo.write(positions[2] + servoAdjust[2]); 
      //yawPlatformServo.write(positions[3] + servoAdjust[3]); 
      leftEarYawServo.write(positions[4] + servoAdjust[4]); 
      leftEarPitchServo.write(positions[5] + servoAdjust[5]); 
      rightEarYawServo.write(positions[6] + servoAdjust[6]); 
      rightEarPitchServo.write(positions[7] + servoAdjust[7]);
      delay(servoDelay); 
    }

    for(int i = 0; i <= 7; i++){ //when done, make previous angles the end position of the motion
      prevAngles[i] = positions[i];
    }
  }

  prevPlatformPos[0] = z;
  prevPlatformPos[1] = roll;
  prevPlatformPos[2] = pitch;
  prevPlatformPos[3] = yaw;
}

void unitCubicCreate(double tf){
  double a, b; //a is cubic coeff, b is squared coeff

  a = -2 / pow(tf, 3);
  b = 3 / pow(tf, 2);

  unitCubicCoeff[0] = a;
  unitCubicCoeff[1] = b;
  
}

void angleCalculate(int arr[3], double z, double roll, double pitch, double yaw){
  // Convert angles to rad
  pitch = pitch * PI / 180.0;
  roll = roll * PI / 180.0;
  yaw = -1 * yaw * PI / 180.0;
  double platformAngle[3], x[3], y[3], linkAngle[3], servoAngle[3];
  double a, b,g, h, z0;
  a = 22.47;
  b = 25;
  g = 45;
  h = 66.138; //was supposed to be 64, servo center to close hole rn is 10.5, should be 9.1 to be accurate
  z0 = 23.103;

  //0 is front, 1 is left, 2 is right
  for(int i = 0; i < 3; i++){
    platformAngle[i] = pitch * cos(yaw + (120 * PI * i /180)) - roll * sin(yaw + (120 * PI * i /180));
    x[i] = (2.0 / 3.0) * (h - g * cos(platformAngle[i]));
    y[i] = (2.0 * g / 3.0) * sin(platformAngle[i]) + z0 + z;
  }

  for(int i = 0; i < 3; i++){
    linkAngle[i] = -acos((pow(x[i], 2) + pow(y[i], 2) - pow(a, 2) - pow(b, 2)) / ( 2 * a * b));
    servoAngle[i] =  180/PI * (atan(y[i] / x[i]) + atan((b * sin(linkAngle[i])) / (a + b * cos(linkAngle[i]))));
    
    if( servoAngle[i] < 0){
      servoAngle[i] = 180 + servoAngle[i];
    }
    if( servoAngle[i] < 90){
      servoAngle[i] = 90 + servoAngle[i];
    }
    else if(servoAngle[i] > 90){
      servoAngle[i] = -90 + servoAngle[i];
    }

    arr[i] = servoAngle[i];
    /*.print(servoAngle[i]);
    Serial.print(" , ");
    Serial.print(x[i]);
    Serial.print(" , ");
    Serial.print(y[i]);
    Serial.println(" , ");*/
  }
}

void testHeadLoop(){
  iterations = 100;
  servoDelay = 2;
  //runPlatform(0, 0, 0, 0, 0); //z, roll, pitch, yaw, no yaw iterations
  
  /*runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 30, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 30, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 60, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90-60, 90+60, 90+60, 90-60, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations*/

  runHead(0, 0, 30, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 30, 90, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 30, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations

  /*runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 35, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, -35, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, -35, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 35, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, -25, -25, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 0, 0, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations
  runHead(0, 25, 25, 0, 90, 90, 90, 90, iterations); //z, roll, pitch, yaw, LEY, LEP, REY, REP, iterations*/
  
  /*leftEarYawServo.write(180); 
  leftEarPitchServo.write(180); 
  rightEarYawServo.write(180); 
  rightEarPitchServo.write(180); 
  delay(500);
  yawPlatformServo.write(90);
  leftEarYawServo.write(90); 
  leftEarPitchServo.write(90); 
  rightEarYawServo.write(90); 
  rightEarPitchServo.write(90); 
  delay(500);
  yawPlatformServo.write(0);
  leftEarYawServo.write(0); 
  leftEarPitchServo.write(0); 
  rightEarYawServo.write(0); 
  rightEarPitchServo.write(0); 
  delay(500);
  yawPlatformServo.write(90);
  leftEarYawServo.write(90); 
  leftEarPitchServo.write(90); 
  rightEarYawServo.write(90); 
  rightEarPitchServo.write(90); 
  delay(500);*/

  /*frontPlatformServo.write(90 + servoAdjust[0]); 
  leftPlatformServo.write(90 + servoAdjust[1]); 
  rightPlatformServo.write(90 + servoAdjust[2]); 
  yawPlatformServo.write(90 + servoAdjust[3]); 
  leftEarYawServo.write(90 + servoAdjust[4]); 
  leftEarPitchServo.write(90 + servoAdjust[5]); 
  rightEarYawServo.write(90 + servoAdjust[6]); 
  rightEarPitchServo.write(90 + servoAdjust[7]);
  delay(500);
  frontPlatformServo.write(110 + servoAdjust[0]); 
  leftPlatformServo.write(110 + servoAdjust[1]); 
  rightPlatformServo.write(110 + servoAdjust[2]); 
  yawPlatformServo.write(110 + servoAdjust[3]); 
  leftEarYawServo.write(110 + servoAdjust[4]); 
  leftEarPitchServo.write(110 + servoAdjust[5]); 
  rightEarYawServo.write(110 + servoAdjust[6]); 
  rightEarPitchServo.write(110 + servoAdjust[7]);
  delay(500);*/
}
