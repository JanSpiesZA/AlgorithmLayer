//This class is used to simulate the ultrasonic sensors

class Sensor
{
  int sensorXPos = 0;      //x position relative to robot centre point
  int sensorYPos = 0;      //y position relative to robot center point
  float sensorHAngle = 0.0;    //Angle of sensor in horizontal plane relative to robot chassis. IE rotated left or right
  float sensorVAngle = 0.0;    //Angle of sensor in vertical plane relative to robot chassis. IE tilted up or down
  float sensorGain = 1.0;      //Gains used to indicate importance of sensor
  float sensorObstacleDist = 0;  //Distance from THIS sensor to obstacle
  float sensorMaxDetect = 200;
  float sensorMinDetect = 0;
  float sensorNoise = 5.0;
  
  Sensor(int _sensorXPos, int _sensorYPos, float _sensorHAngle)
  {
    sensorXPos = _sensorXPos;
    sensorYPos = _sensorYPos;
    sensorHAngle = _sensorHAngle;
  }
  
  Sensor(int _sensorXPos, int _sensorYPos, float _sensorHAngle, float _sensorNoise)
  {
    sensorXPos = _sensorXPos;
    sensorYPos = _sensorYPos;
    sensorHAngle = _sensorHAngle;
  }
  
////////////////////////////////////////////////////////////////////////////////////////////  
  //Displays a sensor's physical placement on the robot chassis based on the reference X, Y and heading values of the sensor
  void display(float _refXPos, float _refYPos, float _refHeading)
  {        
    PVector returnVal = transRot(_refXPos, _refYPos, _refHeading, sensorXPos, sensorYPos);    //Takes the sensor's x,y and plot it in the global frame    
    ellipse(toScreenX(returnVal.x), toScreenY(returnVal.y), 3 * scaleFactor, 3 * scaleFactor);
  }

////////////////////////////////////////////////////////////////////////////////////////////
  //Display the sensors max distance and obstacles detected by sensors
  void displaySensorData(float _refXPos, float _refYPos, float _refHeading)
  {  
    PVector returnVal = transRot(sensorXPos, sensorYPos, sensorHAngle, sensorObstacleDist, 0);    //Takes the sensor's x,y and plot it in the global frame
    returnVal = transRot(_refXPos, _refYPos, _refHeading, returnVal.x, returnVal.y);    //Takes the sensor's x,y and plot it in the global frame
    fill(255);
    stroke(0);
    strokeWeight(1);
    ellipse(toScreenX(int(returnVal.x)), toScreenY(int(returnVal.y)), 10*scaleFactor,10*scaleFactor);
  }
  
////////////////////////////////////////////////////////////////////////////////////////////  
  //Senses if an obstacle is within its cone of detection
  void sense(float _refXPos, float _refYPos, float _refHeading)
  {
    boolean obstacleFlag = false;
    
    sensorObstacleDist = sensorMinDetect;          //Simulate sensor dead zone    
    
    float heading = _refHeading + sensorHAngle;
     
    while ((obstacleFlag == false) && (sensorObstacleDist < sensorMaxDetect))
    { 
      float dX = cos(heading) * sensorObstacleDist;
      float dY = sin(heading) * sensorObstacleDist;
      
      //Tests for USER and MAP tiletypes in order to calculate distance for US sensors to obstacles
      if ((tile[int((_refXPos + dX)/tileSize)][int((_refYPos + dY)/tileSize)].tileType == "MAP") ||
          (tile[int((_refXPos + dX)/tileSize)][int((_refYPos + dY)/tileSize)].tileType == "USER"))
      {
       obstacleFlag = true;
      }
      sensorObstacleDist += 1;      
    }
    
    //Add noise to the measured sensor distance
    sensorObstacleDist += randomGaussian() * sensorNoise;
  }
}