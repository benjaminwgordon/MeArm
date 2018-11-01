//TODO: test angleA and angleB equations for accuracy
//TODO: implement turret rotation and claw movement
//TODO: create limits on servo movement ranges
//TODO: create a goal and implement a controller
#include <Servo.h>

float MINX = -50;
float MAXX = 50;
float MINY = 40;
float MAXY = 100;

int turretServoPin = 9;
int leftArmServoPin = 8;
int rightArmServoPin = 11;
int clawServoPin = 10;

Servo turret;
Servo leftArm;
Servo rightArm;
Servo claw;

float l;
int iterator;
float drawHeight = 0.0;
int interpolationSteps = 20;

void setup() {
  // put your setup code here, to run once:
  turret.attach(turretServoPin);
  leftArm.attach(leftArmServoPin);
  rightArm.attach(rightArmServoPin);
  claw.attach(clawServoPin);
  l = 80;
  Serial.begin(9600);
  iterator = 0;
}

void loop() {
  if (iterator < 5){
    drawNSidedPolygon(iterator + 3, drawHeight, interpolationSteps, iterator * 5);
    iterator++;
  }
}

//draws an regular polygon centered on (x,y) with n sides, and radius r
void drawNSidedPolygon(int n, float drawHeight, int interpolationSteps, float r, float x, float y){
  int polygonXcoords[n];
  int polygonYcoords[n];
  //calculate all x and y coords for regular polygon of radius 1, n sides, centered at (0,0)
  for (int i = 0; i < n; i++){
    polygonXcoords[i] = (sin((float) i / n * 2 * 3.1415926535) * r) + x;
    polygonYcoords[i] = (cos((float) i / n * 2 * 3.1415926535) * r) + y;
  }

  //check for coordinates outside the drawing area
  bool validCoords = true;
  for (int i = 0; i < n; i++){
    if (polygonXcoords[i] > MAXX || polygonXcoords[i] < MINX){
      String errorMessage = "X coordinate at index " + (String) i + " out of range: (X = " + (String) polygonXcoords[i] + ", but X range is [" + (String) MINX + "," + (String) MAXX + "]";
      Serial.print(errorMessage);
      validCoords = false;
    }
    if (polygonYcoords[i] > MAXY || polygonYcoords[i] < MINY){
      String errorMessage = "Y coordinate at index " + (String) i + " out of range: (Y = " + (String) polygonYcoords[i] + ", but Y range is [" + (String) MINY + "," + (String) MAXY + "]";
      Serial.print(errorMessage);
      validCoords = false;
    }
  }
  
  //draw the lines of each side of the polygon, only if coordinates have been verified drawable
  if (validCoords){
    for (int i = 0; i < n - 1; i++){
      cartesianInterpolate(polygonXcoords[i], polygonXcoords[i+1], polygonYcoords[i], polygonYcoords[i+1], drawHeight, drawHeight, interpolationSteps);
    }
    //return to starting pos
    cartesianInterpolate(polygonXcoords[n - 1], polygonXcoords[0], polygonYcoords[0], polygonYcoords[n-1], drawHeight, drawHeight, interpolationSteps);
  }
}

//calculates the angle value for the right arm based on radius and height
int setRight(float rad, float hi) //A, determines rad? or 
{
  float angle;
  int d = hi*hi + rad * rad;
  angle =    180- ((acos((d/(2.0*l*l))-1)+2*atan(hi/rad))/2)*(180/3.1415);
  return (int)angle;
}

//calculates the angle for the left arm based on radius and height
int setLeft(float rad, float hi) //B, determines h? or 
{
  float angle;
  int d = hi * hi + rad * rad;
  angle =  (((acos((d/(2.0*l*l))-1) - 2 * atan(hi/rad)))/2)*(180/3.1415);
  return (int)angle;
}

//interpolates between two points defined in cartesian coordinates to create a smooth line in between
void cartesianInterpolate(float startX, float endX, float startY, float endY, float startZ, float endZ, float interpolationSteps){
  float progress;
  float currentX = startX;
  float currentY = startY;
  float currentZ = startZ;
  for(int i = 0; i <= interpolationSteps; i++){
    currentX += (endX - startX) / interpolationSteps;
    currentY += (endY - startY) / interpolationSteps;
    currentZ += (endZ - startZ) / interpolationSteps;
    cartesianMoveTo(currentX, currentY, currentZ);
    wait(15);
  }
}

//moves the arm to a position specified in cartesian coordinates
void cartesianMoveTo(float x, float y, float z){
  turret.write(cartesianToPolarTheta(x, y));
  leftArm.write(setLeft(cartesianToPolarR(x, y), z));
  rightArm.write(setRight(cartesianToPolarR(x,y), z));
}

//calculates the position of the R value in polar coordinates based on cartesian coordinates
float cartesianToPolarR(float x, float y){
  float R = sqrt(x*x + y*y);
  return R;
}

//calculates the rotation of the turret based on cartesian coordinates
float cartesianToPolarTheta(float x, float y){
  return atan(y/x);
}
