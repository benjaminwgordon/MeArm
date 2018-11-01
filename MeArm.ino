//TODO: test angleA and angleB equations for accuracy -- AARON
//TOOD: implement adjustment factors for arm angle calculations -- AARON



//DONE: implement cartesian coordinate movement -- BEN
//Done: implement an interpolation function to draw smooth lines -- BEN
//DONE: create n sided polygon drawing algorithm -- BEN
//DONE: create limits on servo movement ranges -- BEN
//DONE: create a main loop controller -- BEN
//DONE: implement algorithm for calculating servo angles -- AARON
//DONE: implement method for claw to accept and release pen -- BEN


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
float penSize = 10;

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
  if (iterator == 0)
  {
    //move to edge of table and wait to accept pen
    turret.write(180);
    clawOpenAndShut(penSize);
  }
  if (iterator < 5){
    drawNSidedPolygon(iterator + 3, drawHeight, interpolationSteps, iterator * 5, MINX + (iterator * 15), (MAXY - MINY) / 2.0);
    iterator++;
  }

  if (iterator == 6){
    //rapid move to edge of table and release pen
    turret.write(180);
    clawOpenAndShut(penSize);
  }
}

//draws an regular polygon centered on (x,y) with n sides, and radius r
void drawNSidedPolygon(int n, float drawHeight, int interpolationSteps, float r, float x, float y){
  int polygonXcoords[n];
  int polygonYcoords[n];
  //calculate all x and y coords for regular polygon of radius 1, n sides, centered at (0,0)
  for (int i = 0; i < n; i++){
    polygonXcoords[i] = (sin((float) i / n * 2.0 * 3.1415926535) * r) + x;
    polygonYcoords[i] = (cos((float) i / n * 2.0 * 3.1415926535) * r) + y;
  }

  //check for coordinates outside the drawing area
  bool validCoords = true;
  for (int i = 0; i < n; i++){
    if (polygonXcoords[i] > MAXX || polygonXcoords[i] < MINX){
      String errorMessage = "X coordinate at index " + (String) i + " out of range: (X = " + (String) polygonXcoords[i] + ", but X range is [" + (String) MINX + "," + (String) MAXX + "]";
      Serial.println(errorMessage);
      validCoords = false;
    }
    if (polygonYcoords[i] > MAXY || polygonYcoords[i] < MINY){
      String errorMessage = "Y coordinate at index " + (String) i + " out of range: (Y = " + (String) polygonYcoords[i] + ", but Y range is [" + (String) MINY + "," + (String) MAXY + "]";
      Serial.println(errorMessage);
      validCoords = false;
    }
  }
  
  //draw the lines of each side of the polygon, only if coordinates have been verified drawable
  if (validCoords){
    //move pen to above the starting position
    cartesianMoveTo(polygonYcoords[0], polygonXcoords[0], drawHeight + 20);
    //move pen down to table
    cartesianMoveto(polygonYcoords[0], polygonXcoords[0], drawHeight);

    //draw each line
    for (int i = 0; i < n - 1; i++){
      cartesianInterpolate(polygonXcoords[i], polygonXcoords[i+1], polygonYcoords[i], polygonYcoords[i+1], drawHeight, drawHeight, interpolationSteps);
    }
    //return to starting pos
    cartesianInterpolate(polygonXcoords[n - 1], polygonXcoords[0], polygonYcoords[0], polygonYcoords[n-1], drawHeight, drawHeight, interpolationSteps);
  }
  
  //lift pen
  cartesianMoveTo(polygonYcoords[0], polygonXcoords[0], drawHeight + 20);
}

//calculates the angle value for the right arm based on radius and height
int setRight(float rad, float hi) //A, determines rad? or 
{
  float angle;
  int d = hi*hi + rad * rad;
  angle =    180- ((acos((d/(2.0*l*l))-1)+2*atan(hi/rad))/2.0)*(180/3.1415);
  return (int)angle;
}

//calculates the angle for the left arm based on radius and height
int setLeft(float rad, float hi) //B, determines h? or 
{
  float angle;
  int d = hi * hi + rad * rad;
  angle =  (((acos((d/(2.0*l*l))-1) - 2 * atan(hi/rad)))/2.0)*(180/3.1415);
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
    delay(15);
  }
}


//opens and shuts the claw jaws for a user to place a pen inside
void clawOpenAndShut(float penSize){
  delay(2000);
  //open claw from shut angle to fully open to accept pen
  for (int i = penSize; i < clawOpenAngle; i++){
    claw.write(i);
  }
  //wait for pen to be placed inside jaws
  delay(5000);
  //clamp down around pen
  for (int i = clawOpenAngle; i > penSize; i--){
    claw.write(i);
  }
  //wait for user to move hand away
  delay(2000);
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
