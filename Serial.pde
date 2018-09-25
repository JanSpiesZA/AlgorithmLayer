import processing.serial.*;
Serial motorPort;
Serial usPort;

int lf = 10;
int cr = 13;

String inData = null;
String[] list = null;
String[] list2 = null;

void requestSerialPosition()
{
  println("REQUESTING!!!");
  motorPort.write("<?" + "\r");  
}


//###Sends new velocity and heading data to driver layer
void updateRobot(float _velocityToGoal, float _moveAngle)
{
  //###Limits max velocity and moveAngle to between
  _velocityToGoal *= 10.0;
  
  _velocityToGoal = constrain (_velocityToGoal, -200, 200);  
  //_moveAngle = constrain (_moveAngle, -0.5, 0.5);
  
  //println("TXING!!!");
  String tempAngle = nf(_moveAngle,1,2);
  _moveAngle = float(tempAngle);
  _moveAngle *= 1000;
  
  println(_velocityToGoal+"\t"+_moveAngle);
  motorPort.write("<w"+str(_moveAngle)+"\r");  
  delay(1);
  motorPort.write("<v"+str(_velocityToGoal)+"\r");  
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
        println(inData);
        list = split(inData, ",");        
        //Add robot real world position to inital robot position
        //Divide by 10 to convert mm's into cm's
        //myRobot.location.x = (float(list[0])/10 + robotPosOffset.x); // * scaleFactor;
        //myRobot.location.y = (float(list[1])/10 + robotPosOffset.y); // * scaleFactor;
        
        
        //Convert robot local frame into world frame
        myRobot.location = transRot(robotPosOffset.x, robotPosOffset.y, robotPosOffset.z, float(list[0])/10.0, float(list[1])/10.0);
        myRobot.heading = (float(list[2]) + robotPosOffset.z); // * scaleFactor;
        
        //float delta_x = float(list[0])/10.0*cos(float(list[1])+myRobot.heading);
        //float delta_y = float(list[0])/10.0*sin(float(list[1])+myRobot.heading);
        
        //myRobot.location.x += delta_x;
        //myRobot.location.y += delta_y;
        //myRobot.heading += float(list[1]);
                
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
        for (int cnt = 0; cnt < list.length; cnt++)
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
  try
  {
    inData = p.readString();
    //println(inData);
    //inData = "d0:60,1:60,2:60,3:60,4:60,5:60,6:60" + '\r';
    inData = trim(inData);      //Removes whitespace and carriage return, etc from string
    parseSerialData();
  }
  catch(RuntimeException e)
  {
    e.printStackTrace();
  }
}
