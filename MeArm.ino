//TODO: test angleA and angleB equations for accuracy
//TODO: implement turret rotation and claw movement
//TODO: create limits on servo movement ranges
//TODO: create a goal and implement a controller
#include <Servo.h>

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
  Serial.begin(9600);
}

void loop() {
  //tester code
  polarR = 40;
  polarH = 40;
 
  angleA = polarToServoAngleA(polarR, polarH);
  angleB = polarToServoAngleB(polarR, polarH);
  moveArm(angleA, angleB);
  
}

void moveArm(int angleA, int angleB){
  rightArm.write(angleA);
  leftArm.write(angleB);
  //turret.write(angleC);
  //claw.write(angleD);
}

int polarToServoAngleA(double r, double h){
  double lengthArm = 80;
  double dSquared = (r*r)+(h*h);
  String dSquaredString = "dSquared = " + (String) dSquared;
  Serial.print(dSquaredString);

  double firstTermInternal = dSquared / (2 * lengthArm * lengthArm);
  double firstTerm = acos(firstTermInternal);
  double secondTermInternal = h / r;
  double secondTerm = atan(secondTermInternal);
  String debug = "First term internal = " + (String) firstTermInternal + "   Acos first term = " + (String) firstTerm + "  Second Term Internal = " + (String) secondTermInternal + "  acos second term = " + (String) secondTerm;
  Serial.print(debug);
  
  int A = firstTerm + secondTerm;  
  return A;
}
int polarToServoAngleB(double r, double h){
  double dSquared = (r*r)+(h*h);
  double lengthArm = 80;
  double firstTermInternal = dSquared / (2 * lengthArm * lengthArm);
  double firstTerm = acos(firstTermInternal);
  double secondTermInternal = h / r;
  double secondTerm = atan(secondTermInternal);
  
  int B = firstTerm - secondTerm;  
  return B;
}

void cartesianMoveTo(float x, float y, float z){
  turret.write(cartesianToPolarTheta(x, y));
  leftArm.write(polarToServoAngleB(cartesianToPolarR(x, y), z));
}

float cartesianToPolarR(float x, float y){
  float R = sqrt(x*x + y*y);
  return R;
}

float cartesianToPolarTheta(float x, float y){
  return atan(y/x);
}
