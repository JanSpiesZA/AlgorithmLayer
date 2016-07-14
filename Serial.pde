import processing.serial.*;
Serial myPort;

int lf = 10;
int cr = 13;

String inData = null;
String[] list = null;

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
        myRobot.location.x = (float(list[0])/10.0 + robotStart.x); // * scaleFactor;
        myRobot.location.y = (float(list[1])/10.0 + robotStart.y); // * scaleFactor;
        myRobot.heading = (float(list[2]) +robotStart.z); // * scaleFactor;         
        break;
      }
      
      case 's':
      {
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