#include <Servo.h> // servo library
#include <AccelStepper.h>

//Create objects
Servo myservo; //front
Servo myservo2; //left
Servo myservo3; //right

#define DIR 4
#define STEP 5
#define HALL 13

// Define a stepper and the pins it will use
AccelStepper stepper(1, STEP, DIR); //1 = driver

double array1[4][5];
int angles[3]; //0-front, 1-left, 2-right
int prevAngles[4];
double timeArr[4];
double currentYaw = 0;
double prevPos[3]; //0-z, 1-roll, 2-pitch
double servoDelay;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(HALL, INPUT);
  
  myservo.attach(10); //front
  myservo2.attach(6); //left
  myservo3.attach(9); //right

  myservo.write(90);
  myservo2.write(90);
  myservo3.write(90);
  delay(1000);

  prevAngles[0] = prevAngles[1] = prevAngles[2] = 90;
  prevAngles[3] = 0;
  
  prevPos[0] = prevPos[1] = prevPos[2] = 0;

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(1000);

  homePos();
  
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

  /*angleCalculate(angles, 0, 0, -10, 0); //z, roll, pitch, yaw
  myservo.write(angles[0]);
  myservo2.write(angles[1]);
  myservo3.write(angles[2]);
  delay(500);
  
  for(int i = 0; i < 200; i++){
    stepper.rotate(1.8);
    angleCalculate(angles, 0, 0, -10+20*sin(i*PI/100), (1.8+i*1.8)); //z, roll, pitch, yaw
    myservo.write(angles[0]);
    myservo2.write(angles[1]);
    myservo3.write(angles[2]);
    delayMicroseconds(800);
  }
  delay(2000);*/
  
  int iterations = 55;
  servoDelay = 1;
  double rpm = 50;
  runPlatform(0, 0, 0, 360, 0, rpm); //z, roll, pitch, yaw, no yaw iterations, target rpm
  runPlatform(0, 0, 0, 0, 0, rpm); //z, roll, pitch, yaw, no yaw iterations, target rpm
  runPlatform(0, 0, 20, 450, 0, rpm); //z, roll, pitch, yaw, no yaw iterations, target rpm
  runPlatform(0, 0, 20, -90, 0, rpm); //z, roll, pitch, yaw, no yaw iterations, target rpm
  runPlatform(15, 0, 0, 0, 0, rpm); //z, roll, pitch, yaw, no yaw iterations, target rpm
  runPlatform(0, 0, 0, 180, 0, rpm); //z, roll, pitch, yaw, no yaw iterations, target rpm
  runPlatform(0, 0, 20, 180, iterations, rpm); //z, roll, pitch, yaw, no yaw iterations, target rpm
  runPlatform(-4, 15, 0, 180, iterations, rpm); //z, roll, pitch, yaw, no yaw iterations, target rpm
  runPlatform(0, -8, -12, 180, iterations, rpm); //z, roll, pitch, yaw, no yaw iterations, target rpm
  runPlatform(10, 15, -7, 180, iterations, rpm); //z, roll, pitch, yaw, no yaw iterations, target rpm
  runPlatform(0, 0, 0, 180, iterations, rpm); //z, roll, pitch, yaw, no yaw iterations, target rpm
}

void homePos(){
  /*stepper.setRPM(20);
  int homeSteps = 1;
  
  while(digitalRead(HALL)){
    stepper.rotate(0.9);
    //delayMicroseconds(20);
  }
  
  delay(250);
  stepper.setRPM(15);
  stepper.rotate(homeSteps*0.9);

  delay(400);*/
}

void runPlatform(double z, double roll, double pitch, double yaw, double iters, double rpm){
  int yawDir;
  double deltaZ, deltaRoll, deltaPitch;
  double deltaYaw = yaw - currentYaw;
  int steps = abs(deltaYaw)/0.9;
  double ratio;
  int track = currentYaw;

  if(steps > 0){
    /*if(deltaYaw > 0){
      yawDir = 1;
    }
    else{
      yawDir = -1;
    }*/
    
    /*for(int i = 0; i < steps; i++){
      
      double ratio = (i+1) / (double)steps;
          
      stepper.rotate(yawDir * 0.9);
      currentYaw += yawDir * 0.9;
      //Serial.println(stepper.position());
      
      deltaZ = z * ratio + prevPos[0] * (1 - ratio);
      deltaRoll = roll * ratio + prevPos[1] * (1 - ratio);
      deltaPitch = pitch * ratio + prevPos[2] * (1 - ratio);
      angleCalculate(angles, deltaZ, deltaRoll, deltaPitch, currentYaw); //z, roll, pitch, yaw
      myservo.write(angles[0]);
      myservo2.write(180 - angles[1]);
      myservo3.write(angles[2]);
      //delayMicroseconds(300);
    }*/
    stepper.moveTo(yaw*1600/360);
    
    while(stepper.distanceToGo() != 0){
      ratio = 1 - (stepper.distanceToGo() * 360/1600) / (double)deltaYaw;
      stepper.run();
      currentYaw = stepper.currentPosition() * 360/1600;

      //if((abs(currentYaw - track)) > 8){
        deltaZ = z * ratio + prevPos[0] * (1 - ratio);
        deltaRoll = roll * ratio + prevPos[1] * (1 - ratio);
        deltaPitch = pitch * ratio + prevPos[2] * (1 - ratio);
        angleCalculate(angles, deltaZ, deltaRoll, deltaPitch, currentYaw); //z, roll, pitch, yaw
        myservo.write(angles[0]);
        myservo2.write(180 - angles[1]);
        myservo3.write(angles[2]);
        //track = currentYaw;
      //}
    }
    //delay(100);
    currentYaw = yaw;
  }
  
  else{
    for(int i = 0; i < iters; i++){
      ratio = (i+1) / (double)iters;
      deltaZ = z * ratio + prevPos[0] * (1 - ratio);
      deltaRoll = roll * ratio + prevPos[1] * (1 - ratio);
      deltaPitch = pitch * ratio + prevPos[2] * (1 - ratio);
      angleCalculate(angles, deltaZ, deltaRoll, deltaPitch, currentYaw); //z, roll, pitch, yaw
      myservo.write(angles[0]);
      myservo2.write(180 - angles[1]);
      myservo3.write(angles[2]);
      delay(servoDelay);
    }
    delayMicroseconds(800);
  }

  prevPos[0] = z;
  prevPos[1] = roll;
  prevPos[2] = pitch;
}


void cubicCreate(double arr[4][5], int y0[4], int yf[4], double tf[4]){
  double a, b, d;
  for(int i = 0; i < 4; i++){
    if(yf[i] - y0[i] > 0 || yf[i] - y0[i] < 0){
      a = -2 * (yf[i] - y0[i]) / pow(tf[i], 3);
      b = 3 * (yf[i] - y0[i]) / pow(tf[i], 2);
      d = y0[i];
    }
    else{
      a = b = 0;
      d = yf[i];
    }
    
    arr[i][0] = 0;
    arr[i][1] = a;
    arr[i][2] = b;
    arr[i][3] = d;
    arr[i][4] = tf[i];
    prevAngles[i] = angles[i];
  }
}

void linearCreate(double arr[4][5], int y0[4], int yf[4], double tf[4]){
  double a, b;
  for(int i = 0; i < 4; i++){
    if(yf[i] - y0[i] > 0 || yf[i] - y0[i] < 0){
      a = (yf[i] - y0[i]) / tf[i];
      b = y0[i];
    }
    else{
      a  = 0;
      b = yf[i];
    }

    arr[i][0] = 0;
    arr[i][1] = 0;
    arr[i][2] = a;
    arr[i][3] = b;
    arr[i][4] = tf[i];
    prevAngles[i] = angles[i];
  }
}

void quinticCreate(double arr[4][5], int y0[4], int yf[4], double tf[4]){
  double a, b, c, f;
  for(int i = 0; i < 4; i++){
    if(yf[i] - y0[i] > 0 || yf[i] - y0[i] < 0){
      a = 6 * (yf[i] - y0[i]) / pow(tf[i], 5);
      b = -15 * (yf[i] - y0[i]) / pow(tf[i], 4);
      c = 10 * (yf[i] - y0[i]) / pow(tf[i], 3);
      f = y0[i];
    }
    else{
      a = b = c = 0;
      f = yf[i];
    }

    arr[i][0] = a;
    arr[i][1] = b;
    arr[i][2] = c;
    arr[i][3] = f;
    arr[i][4] = tf[i];
    prevAngles[i] = angles[i];
  }
}

void runLFunction(double arr[4][5], int delayt){
  int longtime;
  int pos[4];

  if(arr[0][4] >= arr[1][4] && arr[0][4] >= arr[2][4]){
    longtime = arr[0][4];
  }
  else if(arr[1][4] >= arr[0][4] && arr[1][4] >= arr[2][4]){
    longtime = arr[1][4];
  }
  else if(arr[2][4] >= arr[0][4] && arr[2][4] >= arr[1][4]){
    longtime = arr[2][4];
  }
  else{
    longtime = arr[0][4];
  }

  for(int i = 0; i <= longtime; i++){
    for(int j = 0; j < 4; j++){
      pos[j] = arr[j][2] * i + arr[j][3];

    }
    
    myservo.write(pos[0]);
    myservo2.write(pos[1]);
    myservo3.write(pos[2]);
    //myservo4.write(pos[3]);
    delay(delayt);
  }
}

void runCFunction(double arr[4][5], int delayt){
  int longtime;
  int pos[4];

  if(arr[0][4] >= arr[1][4] && arr[0][4] >= arr[2][4]){
    longtime = arr[0][4];
  }
  else if(arr[1][4] >= arr[0][4] && arr[1][4] >= arr[2][4]){
    longtime = arr[1][4];
  }
  else if(arr[2][4] >= arr[0][4] && arr[2][4] >= arr[1][4]){
    longtime = arr[2][4];
  }
  else{
    longtime = arr[0][4];
  }

  for(int i = 0; i <= longtime; i++){
    for(int j = 0; j < 4; j++){
      pos[j] = arr[j][1] * pow(i,3) + arr[j][2] * pow(i,2) + arr[j][3];
      
    }
    
    myservo.write(pos[0]);
    myservo2.write(pos[1]);
    myservo3.write(pos[2]);
    //myservo4.write(pos[3]);
    delay(delayt);
  }
  
}

void runQFunction(double arr[4][5], int delayt){
  int longtime;
  int pos[4];

  if(arr[0][4] >= arr[1][4] && arr[0][4] >= arr[2][4]){
    longtime = arr[0][4];
  }
  else if(arr[1][4] >= arr[0][4] && arr[1][4] >= arr[2][4]){
    longtime = arr[1][4];
  }
  else if(arr[2][4] >= arr[0][4] && arr[2][4] >= arr[1][4]){
    longtime = arr[2][4];
  }
  else{
    longtime = arr[0][4];
  }

  for(int i = 0; i <= longtime; i++){
    for(int j = 0; j < 4; j++){
      pos[j] = arr[j][0] * pow(i,5) + arr[j][1] * pow(i,4) + arr[j][2] * pow(i,3) + arr[j][3];
      
    }
    
    myservo.write(pos[0]);
    myservo2.write(pos[1]);
    myservo3.write(pos[2]);
    //myservo4.write(pos[3]);
    delay(delayt);
  }
  
}

void angleCalculate(int arr[3], double z, double roll, double pitch, double yaw){
  pitch = pitch * PI / 180.0;
  roll = roll * PI / 180.0;
  yaw = yaw * PI / 180.0;
  double platformAngle[3], x[3], y[3], linkAngle[3], servoAngle[3];
  double a, b,g, h, z0;
  a = 22.47;
  b = 25;
  g = 45;
  h = 64;
  z0 = 23.103;

  //0 is front, 1 is left, 2 is right
  for(int i = 0; i < 3; i++){
    stepper.run();
    platformAngle[i] = pitch * cos(yaw + (120 * PI * i /180)) - roll * sin(yaw + (120 * PI * i /180));
    x[i] = (2.0 / 3.0) * (h - g * cos(platformAngle[i]));
    y[i] = (2.0 * g / 3.0) * sin(platformAngle[i]) + z0 + z;
  }

  for(int i = 0; i < 3; i++){
    stepper.run();
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
    /*Serial.print(servoAngle[i]);
    Serial.print(" , ");
    Serial.print(x[i]);
    Serial.print(" , ");
    Serial.print(y[i]);
    Serial.println(" , ");*/
  }
}
