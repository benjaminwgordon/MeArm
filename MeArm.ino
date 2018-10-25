//TODO: test angleA and angleB equations for accuracy
//TODO: implement turret rotation and claw movement
//TODO: create limits on servo movement ranges
//TODO: create a goal and implement a controller
#include <Servo.h>;

int turretServoPin = 9;
int leftArmServoPin = 8;
int rightArmServoPin = 11;
int clawServoPin = 10;

Servo turret;
Servo leftArm;
Servo rightArm;
Servo claw;

double polarR;
double polarH;
double polarL;

int angleA;
int angleB;
int angleC;
int angleD;

double cartX;
double CartY;
double CartZ;

void setup() {
  // put your setup code here, to run once:
  turret.attach(turretServoPin);
  leftArm.attach(leftArmServoPin);
  rightArm.attach(rightArmServoPin);
  claw.attach(clawServoPin);
  polarL = 80;
}

void loop() {
  //tester code
  polarR = 80;
  polarH = 60;
 
  angleA = polarToServoAngleA(polarR, polarH);
  angleB = polarToServoAngleB(polarR, polarH);
  moveArm(angleA, angleB);
  
}

void moveArm(){
  rightArm.write(angleA);
  leftArm.write(angleB);
  turret.write(angleC);
  claw.write(angleD);
}

int polarToServoAngleA(double r, double h){
  double l = 80;
  double d = sqrt(r*r+h*h);
  int A = (int) (acos((d*d)/(2*l*l)))+(2*(atan(h/r)));
  return A;
}
int polarToServoAngleA(double r, double h){
  double l = 80;
  double d = sqrt(r*r+h*h);
  int B = (int) (acos((d*d)/(2*l*l)))-(2*(atan(h/r)));
  return B;
}
