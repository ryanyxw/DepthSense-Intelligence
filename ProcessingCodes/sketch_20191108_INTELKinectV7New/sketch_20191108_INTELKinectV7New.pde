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

// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];

//Upper and bottom limits
float upperLimit = 1000;
float bottomLimit = -100;
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

void draw() {
  timeCount ++;
  background(0);

  // Get the raw depth as array of integers
  int[] depth = kinect.getRawDepth();

  // We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
  int skip = 3;

  // Translate and rotate
  translate(width/2, height/2, -50);
  rotateY(a);
  
  if (timeCount % 180 == 0){
    a = -a;
    
    //Returns Direction
    String returnString = getDirection();
    println(returnString);
  }
  
  // Nested for loop that initializes x and y pixels and, for those less than the
  // maximum threshold and at every skiping point, the offset is caculated to map
  // them on a plane instead of just a line
  for (int x = 0; x < kinect.width; x += skip) {
    for (int y = 0; y < kinect.height; y += skip) {
      int offset = x + y*kinect.width;

      // Convert kinect data to world xyz coordinate
      int rawDepth = depth[offset];
      if (depthLookUp[rawDepth]< upperLimit && depthLookUp[rawDepth] > bottomLimit){
      PVector v = depthToWorld(x, y, rawDepth);

      stroke(255);
      pushMatrix();
      // Scale up by 200
      float factor = 600;
      translate(v.x*factor, v.y*factor, factor-v.z*factor);
      // Draw a point
      point(0, 0);
      popMatrix();
      }
    }
  }

  // Rotate
  a += 0.01f;
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




//----------------------------------------------------------------------------------




//This function creates an array that stores the actual depth information and returns and a 2D array
public double[][] createDepthArray(){
  double[][] actualDepth = new double[kinect.height][kinect.width];
  int[] rawDepth = kinect.getRawDepth();
  int currentDepth, cols, rows;
  for (int i = 0; i < rawDepth.length; i++){
    currentDepth = rawDepth[i];
    double finalDepth = depthLookUp[currentDepth];
    cols = i % kinect.width;
    rows = i / kinect.width;
    actualDepth[rows][cols] = finalDepth;
  }
  return actualDepth;
}

//This function gets the total depth value for a given range of columns and returns a double
public double getTotalDepthOf(int startCol, int endCol, double[][] actualDepth){
  double totalDepth = 0;
  for (int row = 0; row < kinect.height; row++){
    for (int col = startCol; col < endCol+1; col++){
      totalDepth += actualDepth[row][col];
    }
  }
  return totalDepth;
}


//Finds the longest empty space within the array and returns the position of that array ( the middle)
public int findLongestEmpty(double[] columnDepth, double minDistance){
  int longestStreak = -1;
  int longestStartIndex = -1;
  int longestEndIndex = -1;
  int currentStartIndex = -1;
  int currentStreak = -1;
  boolean isPrevious = false;
  boolean isCurrent = false;
  double currentValue;
  int returnPosition;
//  println("minDistance = " + minDistance);
  for (int i = 0; i < columnDepth.length; i++){ // Finds the longest streak and corresponding position)
    currentValue = columnDepth[i];
    isCurrent = currentValue > minDistance; //Must be farther away than the max distance
//    println("currentValue = " + currentValue);
    if (!isPrevious && isCurrent){
      currentStreak = 1;
      currentStartIndex = i;
      isPrevious = true;
    }
    else if (isPrevious && isCurrent){
      currentStreak += 1;
    }
    else if (isPrevious && !isCurrent){
      isPrevious = false;
//      println(currentStreak);
      if (currentStreak > longestStreak){
        longestStreak = currentStreak;
        longestStartIndex = currentStartIndex;
        longestEndIndex = i;
      }
    }
  }
  returnPosition = (longestStartIndex + longestEndIndex)/2;
//  println("startIndex = " + longestStartIndex + " endIndex = " + longestEndIndex);
  return returnPosition;
}

//This function gets the optimal position to move and translates it to human language, returning a string
public String translateDirection(int actualIndex, int totalIndexes, double turnAngle){
  double compareDecimal = (double)actualIndex / totalIndexes;
//  println("actual index is " + actualIndex);
//  println("final decimal is " +compareDecimal);
  if (compareDecimal < 0.5){
    return "Turn Left By " + turnAngle + "°";
  }
  else {
    return "Turn Right By " + turnAngle + "°";
  }
//  if (compareDecimal <0.2){
 //   return "Turn Very Left";
//  }
//  else if ((compareDecimal >= 0.2) && (compareDecimal < 0.4)){
//    return "Turn Somewhat Left";
//  }
//  else if ((compareDecimal >= 0.4) && (compareDecimal < 0.6)){
//    return "Move Straight Forwards";
//  }
//  else if ((compareDecimal >= 0.6) && (compareDecimal < 0.8)){
//    return "Turn Somewhat Right";
//  }
//  else { //if (compareDecimal >= 0.8)
//    return "Turn Very Right";
//  }
}

//This function completes all necessary steps in order to finally return the directions the individual needs to go to
public String getDirection(){
  double[][] finalDepth = createDepthArray(); //Creates depth array
  int skipCols = 10; // how many columns are grouped into a single group represented as single index in eventual array
//  int pixelsPerColGroup = skipCols * kinect.height;
//  double minimumDepth = 3 * pixelsPerColGroup; //The minimum depth allowed for an object to be considered 'far enough'
  double minimumDepth = 7000.0;
  double[] arraySums = new double[kinect.width/skipCols]; //Stores the summed up depth values for groups of array
  double currentDepth;
  int actualIndex; //The position where you want the person to move towards
  String returnString = ""; //The string you are returning as direction
  
  for (int i = 0; i < kinect.width/skipCols; i ++){ //Iterating through with 'i' representing the number of sets of columns
    actualIndex = i * skipCols;
    currentDepth = getTotalDepthOf(actualIndex, actualIndex + skipCols - 1, finalDepth);
    arraySums[i] = currentDepth;
  }
  
  actualIndex = findLongestEmpty(arraySums, minimumDepth);
  double turnAngle = getDegrees(arraySums.length, actualIndex);
  returnString = translateDirection(actualIndex, kinect.width/skipCols, turnAngle);
  double horizontalDirection = getHorizontalDirection(finalDepth, actualIndex, turnAngle, skipCols);
  double verticalDirection = getVerticalDirection(finalDepth, actualIndex, turnAngle, skipCols);
  returnString = appendDirectionMessage(returnString, horizontalDirection, verticalDirection, arraySums.length/2, actualIndex);
  return returnString;
  
}


//--------------------------------------------------------------------------------------------
//Calculations for directions

//This method gets the number of degrees one should turn
public double getDegrees(int arrayLength, int actualIndex){
  int middleIndex = arrayLength / 2;
  int fromMiddle = abs(actualIndex - middleIndex);
  double returnRadians = atan((fromMiddle * tan(57 * PI/180))/middleIndex);
  double returnDegrees = returnRadians * 180;
  return returnDegrees;
}
/*
//This method returns the row and column values of the largest depth
public int[] getLargestIndex(double[][]actualDepth, int startCol, int endCol){
  int bestRow = -1;
  int bestCol = -1;
  double largestDepth = actualDepth[0][startCol];
  double currentValue;
  for (int row = 0; row < columnDepth.length; row++){
    for (int col = startCol; col < endCol+1; col++){
      currentValue = actualDepth[row][col];
      if (currentValue > largestDepth){
        largestDepth = currentValue;
        bestRow = row;
        bestCol = col;
      }
    }
  }
  int[] returnArray = {bestRow, bestCol};
  return returnArray;
}
*/
//This method gets the linear direction needed to move to final destination
public double getHorizontalDirection(double[][] actualDepth, int actualIndex, double turnAngle, int skipCols){
  int colIndex = actualIndex * skipCols + skipCols / 2;
  double distanceFromTarget = actualDepth[actualDepth.length/2][colIndex];
  float radianAngle = (float)turnAngle * PI/ 180;
  double horizontalMovement = sin(radianAngle) * distanceFromTarget;
  return horizontalMovement;
}

//This method gets the vertical direction needed to move to final destination
public double getVerticalDirection(double[][] actualDepth, int actualIndex, double turnAngle, int skipCols){
  int colIndex = actualIndex * skipCols + skipCols / 2;
  double distanceFromTarget = actualDepth[actualDepth.length/2][colIndex];
  float radianAngle = (float)turnAngle * PI/ 180;
  double verticalMovement = cos(radianAngle) * distanceFromTarget;
  return verticalMovement;
}

//This method returns the final return message of the program that includes the horizontal and vertical
public String appendDirectionMessage(String returnString, double horizontalDirection, double verticalDirection, int middleArraySum, int actualIndex){
  String appendHorizontal = "";
  if (actualIndex < middleArraySum){
    appendHorizontal = "Move Left By " + horizontalDirection + " Meters";
  }
  else{
    appendHorizontal = "Move Right By " + horizontalDirection + " Meters";
  }
  returnString += "\n" + appendHorizontal + "\n" + "Then Move Forward by " + verticalDirection + " Meters";
  return returnString;
}
