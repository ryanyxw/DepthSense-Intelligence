// Daniel Shiffman
// Kinect Point Cloud example

// https://github.com/shiffman/OpenKinect-for-Processing
// http://shiffman.net/p5/kinect/

import org.openkinect.freenect.*;
import org.openkinect.processing.*;

// Kinect Library object
Kinect kinect;

// Angle for rotation
float a = 0;
int origX = 0;
int origY = 0;
int newX = 0;
int newY = 0;
boolean isFirst = true;
int totalXRotate = 0;
int totalYRotate = 0;

// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];



//Upper and bottom limits
float upperLimit = 10;
float bottomLimit = 0;
int timeCount = 0;

void setup() {
  // Rendering in P3D
  size(1200, 800, P3D);
  kinect = new Kinect(this);
  kinect.initDepth();

  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i); // Setting up a measuring array that stores each specific value of real depth
  }
}


void mouseWheel(MouseEvent event){
  totalYRotate += 0.05 * event.getCount();
}
  
void draw() {
  timeCount ++;
  background(0);

//  if(mousePressed ){
    
//  totalXRotate += (0.005*pmouseY);
//  totalYRotate += (0.015*(mouseX - pmouseX));
 
//}
  rotateX(totalXRotate);
  rotateY(totalYRotate);

  // Get the raw depth as array of integers
  int[] depth = kinect.getRawDepth();

  // We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
  int skip = 4;

  // Translate and rotate
  translate(width/2, height/2, -50);
//  rotateY(a);
  
//  if (timeCount % 180 == 0){
//    a = -a;
//  }
  stroke(255);
  strokeWeight(2);
  // Nested for loop that initializes x and y pixels and, for those less than the
  // maximum threshold and at every skiping point, the offset is caculated to map
  // them on a plane instead of just a line
  pushMatrix();
  for (int x = 0; x < kinect.width; x += skip) {
    for (int y = 0; y < kinect.height; y += skip) {
      int offset = x + y*kinect.width;

      // Convert kinect data to world xyz coordinate
      int rawDepth = depth[offset];
      if (depthLookUp[rawDepth]< upperLimit && depthLookUp[rawDepth] > bottomLimit){
      PVector point = depthToWorld(x, y, rawDepth);

      
//      pushMatrix();
      // Scale up by 200
      float factor = 100;
//      translate(v.x*factor, v.y*factor, factor-v.z*factor);
      // Draw a point
      point(point.x * factor, point.y * factor, point.z * factor);
//      popMatrix();
      }
    }
  }
  popMatrix();

  // Rotate
//  a += 0.005f;
}

// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

// Only needed to make sense of the ouput depth values from the kinect
PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

// Drawing the result vector to give each point its three-dimensional space
  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}
