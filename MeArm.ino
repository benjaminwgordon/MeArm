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

int turretServoPin = 9;
int leftArmServoPin = 8;
int rightArmServoPin = 11;
int clawServoPin = 10;

Servo turret;
Servo leftArm;
Servo rightArm;
Servo claw;

int iterator;  //main iterator for loop



//GLOBAL SETTINGS
float l = 80; //length of main arm
float drawHeight = 0.0; //height to draw at
int interpolationSteps = 20;  //number of subdivisions for any line
float penSize = 80; //angle required to hold pen
float clawOpenAngle = penSize + 30; //angle claw opens to to accept pen

float MINX = -50; //millimeters left of turret rotation center that canvas extends
float MAXX = 50; //millimeters right of turret rotation center that canvas extends
float MINY = 30; //millimeters forwards from turret center that canvas begins
float MAXY = 140; //millimeters forwards from turret center that canvas ends


void setup() {
  // put your setup code here, to run once:
  turret.attach(turretServoPin);
  leftArm.attach(leftArmServoPin);
  rightArm.attach(rightArmServoPin);
  claw.attach(clawServoPin);
  Serial.begin(9600);
  iterator = 1;
}

void loop() {
  if (iterator == 1)
  {
    //move to edge of table and wait to accept pen
    turret.write(90);
    claw.write(90);
    delay(1000);
    claw.write(130);
    delay(1000);
    drawNSidedPolygon(4, 0.0, 3, 10, 0, 80);
    iterator++;
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
    Serial.println("Point +" + (String)i + ": (" + (String)polygonXcoords[i] + ", " + (String)polygonYcoords[i] + ")");
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
    cartesianMoveTo(polygonXcoords[0], polygonYcoords[0], drawHeight + 20);

    //draw each line
    for (int i = 0; i < n - 1; i++){
      cartesianInterpolate(polygonXcoords[i], polygonXcoords[i+1],polygonYcoords[i], polygonYcoords[i+1], interpolationSteps);
    }
    //return to starting pos
    cartesianInterpolate(polygonXcoords[n - 1], polygonXcoords[0], polygonYcoords[0], polygonYcoords[n-1], interpolationSteps);
  }
  
  //lift pen
  cartesianMoveTo(polygonYcoords[0], polygonXcoords[0], drawHeight + 20);
}

//calculates the angle value for the right arm based on radius and height
int setRight(float rad, float hi) //A, determines rad? or 
{
  float d = hi*hi + rad * rad;
  float acosInternal  = d / 12800.0 - 1;
  float Acos = acos(acosInternal);
  float atanInternal = hi / rad;
  float Atan = atan(atanInternal);
  float angleRad = (0.5 * Acos) - Atan;
  float angle = angleRad * 180 / 3.1415926535;
  float adjustedAngle = 135 - angle;
//  Serial.println("d=" + (String)d);
//  Serial.println("acosInteral=" + (String)acosInternal);
//  Serial.println("Acos=" + (String)Acos);
//  Serial.println("atanInternal=" + (String)atanInternal);
//  Serial.println("Atan=" + (String)Atan);
//  Serial.println("angle in Rad=" + (String)angleRad);
//  Serial.println("angle in deg=" + (String)angle);  
//  Serial.println("setting right arm to expected real location: " + (String)angle);  
//  Serial.println("setting right arm to servo actual location: " + (String)adjustedAngle); 
  return (int)adjustedAngle;
}

//calculates the angle for the left arm based on radius and height
int setLeft(float rad, float hi) //B, determines h? or 
{
  float d = hi*hi + rad * rad;
  float acosInternal  = d / 12800.0 - 1;
  float Acos = acos(acosInternal);
  float atanInternal = hi / rad;
  float Atan = atan(atanInternal);
  float angleRad = (0.5 * Acos) + Atan;
  float angle = angleRad * 180 / 3.1415926535;
  float adjustedAngle = 180 - 53 - angle;
//  Serial.println("d=" + (String)d);
//  Serial.println("acosInteral=" + (String)acosInternal);
//  Serial.println("Acos=" + (String)Acos);
//  Serial.println("atanInternal=" + (String)atanInternal);
//  Serial.println("Atan=" + (String)Atan);
//  Serial.println("angle in Rad=" + (String)angleRad);
//  Serial.println("angle in deg=" + (String)angle);  
//  Serial.println("setting left arm to expected real location: " + (String)angle);  
//  Serial.println("setting left arm to servo actual location: " + (String)adjustedAngle);  
  return adjustedAngle;
}

//interpolates between two points defined in cartesian coordinates to create a smooth line in between
void cartesianInterpolate(float startX, float endX, float startY, float endY, float interpolationSteps){
  float progress;
  float currentX = startX;
  float currentY = startY;
  float stepX = (endX - startX) / interpolationSteps;
  float stepY = (endY - startY) / interpolationSteps;
  for(int i = 0; i < interpolationSteps; i++){
    currentX += stepX;
    currentY += stepY;
    Serial.println("[" + (String) currentX + ", " + (String) currentY + "]");
    cartesianMoveTo(currentX, currentY, drawHeight + 20);
    int wait = 0;
    while (wait < 10000){
      wait++;
    }
    wait = 0;
    cartesianMoveTo(currentX, currentY, drawHeight);
    while (wait < 10000){
      wait++;
    }
    cartesianMoveTo(currentX, currentY, drawHeight + 20);
    wait = 0;
    while (wait < 10000){
      wait++;
    }
  }
}

//moves the arm to a position specified in cartesian coordinates
void cartesianMoveTo(float x, float y, float z){
  float Theta = cartesianToPolarTheta(x, y);
  float R = cartesianToPolarR(x, y);
  turret.write(Theta);
  leftArm.write(setLeft(R, z));
  rightArm.write(setRight(R, z));
  Serial.println("turret: " + (String)Theta);
  delay(1000);
}

//calculates the position of the R value in polar coordinates based on cartesian coordinates
float cartesianToPolarR(float x, float y){
  float R = sqrt(x*x + y*y);
  return R;
}

//calculates the rotation of the turret based on cartesian coordinates
float cartesianToPolarTheta(float x, float y){
  float Theta =  atan(y/x) * 180/3.1415926535;
  //Serial.println("Atan degrees = " + (String)Theta);
  int complementTheta = (int)((180.0 - Theta) * 10000);
  //Serial.println("complement of theta * 10000 = " + (String)complementTheta);
  float modTheta = 90 + (complementTheta % 1800000) / 10000.0;
  //Serial.println("final angle for turret = " + (String)modTheta);
  return modTheta;
}
