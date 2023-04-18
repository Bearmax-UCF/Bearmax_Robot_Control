#include "body.hpp"

//Servo chassisServo;
Servo lasServo;
Servo larServo;
Servo laeServo;
Servo rasServo;
Servo rarServo;
Servo raeServo;

void setupBody() {
//    chassisServo.attach(CHASSIS_PIN);
    lasServo.attach(RIGHT_ARM_SHOULDER_PIN);
    larServo.attach(RIGHT_ARM_ROTATOR_PIN);
    laeServo.attach(RIGHT_ARM_ELBOW_PIN);
    rasServo.attach(LEFT_ARM_SHOULDER_PIN);
    rarServo.attach(LEFT_ARM_ROTATOR_PIN);
    raeServo.attach(LEFT_ARM_ELBOW_PIN);

 //   chassisServo.write(90);
    lasServo.write(90);
    larServo.write(90);
    laeServo.write(90);
    rasServo.write(90);
    rarServo.write(90);
    raeServo.write(90);
}

void runBody(int chassis, int las, int lar, int lae, int ras, int rar, int rae) {
//    chassisServo.write(chassis + 90);
    lasServo.write(las + 90);
    larServo.write(lar + 90);
    laeServo.write(lae + 90);
    rasServo.write(ras + 90);
    rarServo.write(rar + 90);
    raeServo.write(rae + 90);
}
