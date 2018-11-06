#include <Servo.h>

int turretServoPin = 9;
int leftArmServoPin = 8;
int rightArmServoPin = 11;
int clawServoPin = 10;

Servo turret;
Servo leftArm;
Servo rightArm;

bool run = true;

void setup() {
  // put your setup code here, to run once:
  turret.attach(turretServoPin);
  leftArm.attach(leftArmServoPin);
  rightArm.attach(rightArmServoPin);
  Serial.begin(9600);
}

void loop() {
  while(run){

    //int n, int r, float x, float y, float drawHeight
    nSidedPolygon(4, 30, 0, 100, 0);

    run = false;
  }
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
  return adjustedAngle;
}

//very big method because language doesnt support passing arrays as parameters
void nSidedPolygon(int n, int r, float x, float y, float drawHeight){
  //begin by filling an array with cartesian coordinates corresponding to the corners
  //of a regular n sided polygon with radius r
  float polygonXcoords[n];
  float polygonYcoords[n];
  //calculate all x and y coords for regular polygon of radius 1, n sides, centered at (x,y)
  for (int i = 0; i < n; i++){
    polygonXcoords[i] = (sin((float) (i / (float)n) * 2.0 * 3.1415926535) * r) + x;
    polygonYcoords[i] = (cos((float) (i / (float)n) * 2.0 * 3.1415926535) * r) + y;
  }

  //next, interpolate midpoints for these corners to fill out the shape
  float filledPolygonXcoords[n*2];
  float filledPolygonYcoords[n*2];
  int cornerArrayIndex = 0;
  int filledArrayIndex = 0;
  while (filledArrayIndex < n * 2){
    //add the corners
    filledPolygonXcoords[filledArrayIndex] = polygonXcoords[cornerArrayIndex];
    filledPolygonYcoords[filledArrayIndex] = polygonYcoords[cornerArrayIndex];
    filledArrayIndex++;
    //add the midpoints
    filledPolygonXcoords[filledArrayIndex] = 0.5 * (polygonXcoords[cornerArrayIndex] + polygonXcoords[cornerArrayIndex+1]);
    filledPolygonYcoords[filledArrayIndex] = 0.5 * (polygonYcoords[cornerArrayIndex] + polygonYcoords[cornerArrayIndex+1]);    
    filledArrayIndex++;
    cornerArrayIndex++;
  }
  //wrap back to start of array
  filledPolygonXcoords[n * 2 - 1] = 0.5 * (polygonXcoords[n-1] + polygonXcoords[0]);
  filledPolygonYcoords[n * 2 - 1] = 0.5 * (polygonYcoords[n-1] + polygonYcoords[0]);    
  

  //then, convert these cartesian coordinates to polar coordinates
  float polarRadius[n * 2];
  float polarAngle[n * 2];
  
  //convert all the points to polar coordinates
  for (int j = 0; j < n * 2; j++){
    //sqrt(X*X+Y*Y)
    polarRadius[j] = sqrt((filledPolygonXcoords[j] * filledPolygonXcoords[j])+(filledPolygonYcoords[j] * filledPolygonYcoords[j]));
    //atan(y/x)
    float polarAngleTemp = 180 - ((180 / 3.1415) * atan(filledPolygonYcoords[j] / filledPolygonXcoords[j]));
    if (polarAngleTemp < 0) polarAngleTemp += 180;
    if (polarAngleTemp > 180) polarAngleTemp -= 180;
    polarAngle[j] = polarAngleTemp;
  }
  
  for (int i = 0; i < n * 2; i++){
    Serial.println("point " + (String)i + " at [" + (String)filledPolygonXcoords[i] + ", " + (String)filledPolygonYcoords[i] + "] radius = " + (String)polarRadius[i] + ", angle = " + polarAngle[i]);
  }

  //finally, draw each point:
  for (int i = 0; i < n * 2; i++){
    //move to a point above (x,y)
    turret.write(polarAngle[i]);
    leftArm.write(setLeft(polarRadius[i], drawHeight + 20));
    rightArm.write(setRight(polarRadius[i], drawHeight + 20));
    int wait = 0;
    while (wait < 100){
      wait++;
      Serial.println((String)wait);
    }
    //move z down to draw with the marker
    leftArm.write(setLeft(polarRadius[i], drawHeight));
    rightArm.write(setRight(polarRadius[i], drawHeight));
    wait = 0;
    while (wait < 100){
      wait++;
      Serial.println((String)wait);
    }
    //move z up to clear the paper
    leftArm.write(setLeft(polarRadius[i], drawHeight + 20));
    rightArm.write(setRight(polarRadius[i], drawHeight + 20));   
    wait = 0;
    while (wait < 100){
      wait++;
      Serial.println((String)wait);

    }
  }  
}
