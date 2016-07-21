import processing.serial.*;
Serial myPort;

int lf = 10;
int cr = 13;

String inData = null;
String[] list = null;
String[] list2 = null;

void requestSerialPosition()
{
  println("REQUESTING!!!");
  myPort.write("<?" + "\r");  
}


//###Sends new velocity and heading data to driver layer
void updateRobot(float _velocityToGoal, float _moveAngle)
{
  //###Limits max velocity to between
  _velocityToGoal = constrain (_velocityToGoal, -200, 200);  
  _moveAngle = constrain (_moveAngle, -1, 1);
  
  println("TXING!!!");
  String tempAngle = nf(_moveAngle,1,2);
  _moveAngle = float(tempAngle);
  _moveAngle *= 1000;
  myPort.write("<w"+str(_moveAngle)+"\r");  
  delay(1);
  myPort.write("<v"+str(_velocityToGoal)+"\r");  
}

void parseSerialData()
{
  if (inData != null)
  {
    switch (inData.charAt(0))
    {
      case '?':
      {
        inData = inData.substring(1);
        //println(inData);
        list = split(inData, ",");
        
        //Add robot real world position to inital robot position
        myRobot.location.x = (float(list[0])/10.0 + robotPosOffset.x); // * scaleFactor;
        myRobot.location.y = (float(list[1])/10.0 + robotPosOffset.y); // * scaleFactor;
        myRobot.heading = (float(list[2]) +robotPosOffset.z); // * scaleFactor;         
        break;
      }
      
      case 's':
      {
        break;
      }
      
      case 'd':
      {
        //###Create a substring of inData starting at location 1 in the string
        //###   this removes the very first character from inData
        inData = inData.substring(1);
        list = split(inData, ",");
        for (int cnt = 0; cnt < numSensors2; cnt++)
        {
          list2 = split(list[cnt], ":");
          myRobot.sensors.get(int(list2[0])).sensorObstacleDist = int(list2[1]);
        }        
        break;
      }
    }
   inData = null; 
  }
}

void serialEvent(Serial p)
{
  inData = p.readString();
}