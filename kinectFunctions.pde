
// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}

void drawPixels()
{
 // Get the raw depth as array of integers
 int[] depth = kinect.getRawDepth();
  
 for (int x = 0; x < kinect.width; x += skip) 
 {
   for (int y = 0; y < kinect.height; y += skip) 
   {
    int offset = x + y*kinect.width;

     // Convert kinect data to world xyz coordinate
     int rawDepth = depth[offset];
     PVector v = depthToWorld(x, y, rawDepth);
     
     v.x *= 100;      //Convert depth value from meters into mm
     v.z *= 100;
     
     if (v.z > 0 && v.z < maxKinectDetectNormal)    //Test for any invalid depth values      
     {
        fill(255);
        PVector returnVal = transRot(myRobot.location.x, myRobot.location.y, myRobot.heading - PI/2, v.x, v.z);
        strokeWeight(0);
        ellipse(toScreenX(int(returnVal.x)), toScreenY(int(returnVal.y)), 5 * scaleFactor, 5 * scaleFactor);
        
        if (v.z <= maxKinectPersistantView) updateGravity(returnVal.x, returnVal.y);
     }
   }
 }  
}



//Adds 'n gravity value to the occupancy grid based on the amount of pixels inside each cell
void updateGravity(float _x, float _y)
{  
  int x = int(_x/(tileSize));
  if (x < 0) x = 0;
  if (x >= maxTilesX) x = maxTilesX-1;
  
  int y = int(_y/(tileSize));
  if (y < 0) y = 0;
  if (y >= maxTilesY) y = maxTilesY-1;
  
  if (tile[x][y].tileType == "UNASSIGNED") tile[x][y].tileType = "KINECT";      //Change tileType to KINECT only when tile is not asigned
  tile[x][y].gravity++;
  tile[x][y].update();
}