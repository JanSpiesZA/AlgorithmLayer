/////////////////////////////////////////////////////////////////////////////////////////////////
void detectObstacle()
{ 
  boolean obstacleFlag = false;
  float detectDistance = 0.0;
  float obstacleX = 0.0;
  float obstacleY = 0.0; 

  fill(255);
  stroke(1);
  for (int i = 0; i < numSensors; i++)
  {
    sensorObstacleDist[i] = 2; //myRobot.diameter/2 + 1;    //Set starting point of collision detect to 1 pixel wider than the radius of the robot    
    obstacleFlag = false;

    while ((obstacleFlag == false) && (sensorObstacleDist[i] < maxDetectDistance))
    {
      PVector returnVal = transRot(myRobot.location.x, myRobot.location.y, myRobot.heading, sensorX[i], sensorY[i]);  //translates sensordata to global frame
      obstacleX = returnVal.x + sensorObstacleDist[i] * cos(myRobot.heading + sensorPhi[i]);
      obstacleY = returnVal.y + sensorObstacleDist[i] * sin(myRobot.heading + sensorPhi[i]);   
      color col = get (int(obstacleX), int(obstacleY));    //Test pixel colour to determine if there is an obstacle
      if (col == color(200, 150, 150))                //Test to see if tile is an obstacle
        obstacleFlag = true;
      sensorObstacleDist[i] += 1;
    }
    if (sensorObstacleDist[i] <= safeDistance) myRobot.collisionFlag = true;      //Set collision flag when any sensor is too close to obstacle
    ellipse (obstacleX, obstacleY, 10*scaleFactor, 10*scaleFactor);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//Calculates a vector which points away from obstacles using the distance vectors from all the sensors to obstacles 
void calcVecAO()
{
  float x_vector = 0.0;
  float y_vector = 0.0;
  float x1_vector = 0.0;
  float y1_vector = 0.0;

  vectorAO[0] = 0.0;
  vectorAO[1] = 0.0;

  for (int i=0; i < numSensors; i++)
  {
    PVector returnVal = transRot(myRobot.location.x, myRobot.location.y, myRobot.heading, sensorX[i], sensorY[i]);  //translates sensordata to global frame
    x1_vector = returnVal.x; //x_temp;
    y1_vector = returnVal.y; //y_temp;

    returnVal = transRot (sensorX[i], sensorY[i], sensorPhi[i], sensorObstacleDist[i], 0);    //translates obstacle distance to robot frame
    returnVal = transRot (myRobot.location.x, myRobot.location.y, myRobot.heading, returnVal.x, returnVal.y);  //translates sensordata in robot frame to global frame
    x_vector = returnVal.x - x1_vector;
    y_vector = returnVal.y - y1_vector;

    //Add all the vectors together and multiply with vector gains
    vectorAO[0] += (x_vector * sensorGains[i]);
    vectorAO[1] += (y_vector * sensorGains[i]);  

    //line (myRobot.x, myRobot.y, x_temp, y_temp);
  }
  float n = sqrt(pow(vectorAO[0], 2)+pow(vectorAO[1], 2));    //Calculates the normalisation factor for the AvoidObstacle vector

  vectorAO[0] = 100 * vectorAO[0]/n;      //Multiply by 100 gain in order to control the length of the unity vectors
  vectorAO[1] = 100 * vectorAO[1]/n;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void calcVecGTG()
{
  vectorGTG[0] = goalX - myRobot.location.x;
  vectorGTG[1] = goalY - myRobot.location.y;

  float n = sqrt(pow(vectorGTG[0], 2)+pow(vectorGTG[1], 2));    //Calculates the normailsation factor for the AvoidObstacle vector
  if (n==0) n=1;
  vectorGTG[0] = 100 * vectorGTG[0]/n;      //Multiply by 100 gain in order to control the length of the unity vectors
  vectorGTG[1] = 100 * vectorGTG[1]/n;
  
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void calcVecAO_GTG()
{

  for (int i = 0; i <= 1; i++)
  {
    vectorAO_GTG[i] = blendGain*vectorAO[i] + (1 - blendGain)*vectorGTG[i];
  }

  float n = sqrt(pow(vectorAO_GTG[0], 2)+pow(vectorAO_GTG[1], 2));    //Calculates the normailsation factor for the AvoidObstacle vector

  vectorAO_GTG[0] = 100 * vectorAO_GTG[0]/n;      //Multiply by 100 gain in order to control the length of the unity vectors
  vectorAO_GTG[1] = 100 * vectorAO_GTG[1]/n;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void estimateWall()
{
  int c1 = 0;
  int c2 = 1;

  //Step 1:
  //  Determine the two smallest values from the left sensors
  for (int i = 0; i <= ceil(numSensors/2); i++)
  {
    if (sensorObstacleDist[i] < sensorObstacleDist[c1]) c1 = i;
  }

  for (int i = 0; i <= ceil(numSensors/2); i++)
  {
    if ((sensorObstacleDist[i] < sensorObstacleDist[c2]) && (sensorObstacleDist[i] > sensorObstacleDist[c1])) c2 = i;
  }
  if (c1 == c2) c2 = c1 + 1;

  int leftC1 = min(c1, c2);
  int leftC2 = max(c1, c2);

  c1 = leftC1;
  c2 = leftC2;

  //step 2:
  //  Convert the distances to points in the global frame
  //  Create a wall vector and normalise it
  if ((c1 >= 0) & (c2 >= 0))
  {
    PVector returnVal = transRot (sensorX[c1], sensorY[c1], sensorPhi[c1], sensorObstacleDist[c1], 0);    //translates obstacle distance to robot frame
    returnVal = transRot (myRobot.location.x, myRobot.location.y, myRobot.heading, returnVal.x, returnVal.y);  //translates sensordata in robot frame to global frame
    closest1[0] = returnVal.x;
    closest1[1] = returnVal.y;

    returnVal = transRot (sensorX[c2], sensorY[c2], sensorPhi[c2], sensorObstacleDist[c2], 0);    //translates obstacle distance to robot frame
    returnVal = transRot (myRobot.location.x, myRobot.location.y, myRobot.heading, returnVal.x, returnVal.y);  //translates sensordata in robot frame to global frame
    closest2[0] = returnVal.x;
    closest2[1] = returnVal.y;

    ellipse (closest1[0], closest1[1], 20, 20);
    ellipse (closest2[0], closest2[1], 20, 20);
    //line (closest1[0], closest1[1], closest2[0], closest2[1]);

    vectorWall[0] = closest2[0] - closest1[0];
    vectorWall[1] = closest2[1] - closest1[1];

    float n = sqrt(pow(vectorWall[0], 2)+pow(vectorWall[1], 2));    //Calculates the normalisation factor for the AvoidObstacle vector
    vectorWall[0] = normaliseGain * vectorWall[0]/n;      //Multiply by 100 gain in order to control the length of the unity vectors
    vectorWall[1] = normaliseGain * vectorWall[1]/n;

    //Step 3
    //  Compute a vector perpendicular with the wall pointing from the center of the robot to the wall vector
    //  http://stackoverflow.com/questions/1811549/perpendicular-on-a-line-from-a-given-point

    float k = ((closest2[1] - closest1[1]) * (myRobot.location.x - closest1[0]) - (closest2[0] - closest1[0])*(myRobot.location.y - closest1[1])) / (pow(closest2[1]-closest1[1], 2) + pow(closest2[0]-closest1[0], 2));
    vectorWallDist[0] = myRobot.location.x - k *(closest2[1] - closest1[1]);
    vectorWallDist[1] = myRobot.location.y + k *(closest2[0] - closest1[0]);

    vectorWallDist[0] -= myRobot.location.x;
    vectorWallDist[1] -= myRobot.location.y;

    n = sqrt(pow(vectorWallDist[0], 2)+pow(vectorWallDist[1], 2));    //Calculates the normalisation factor for the AvoidObstacle vector
    /*
     vectorWallDist[0] = normaliseGain * vectorWallDist[0]/n;      //Multiply by 100 gain in order to control the length of the unity vectors
     vectorWallDist[1] = normaliseGain * vectorWallDist[1]/n;
     */
    //Step 4
    //  Determines vector pointing away from the wall and weighted with distanceFromWall
    for (int i = 0; i < 2; i++)
    {
      vectorAwayFromWall[i] =  vectorWallDist[i] - distanceFromWall * vectorWallDist[i]/n;
    }


    //Step 5
    //  Calculates vectorFollowWall used to point the robot in the right direction when following the wall
    for (int i=0; i < 2; i++)
    {
      vectorFollowWall[i] = vectorWall[i] + vectorAwayFromWall[i];
    }
  }
}

//###############################################################################################
//Applies a scale factor to all the values used according to the actual world size and the displayed screen size

//MUST BE CLEANED
void applyScale()
{
  myRobot.robotDiameter *= scaleFactor;
  myRobot.noseLength *= scaleFactor;
  myRobot.maxSpeed *= scaleFactor;
  //myRobot.maxTurnRate *= scaleFactor;
  minDetectDistance *= scaleFactor;        //Closer than this value and the sensors do not return valid data
  maxDetectDistance *= scaleFactor;
  safeZone *= scaleFactor;          //Safe area around target assumed the robot reached its goal;
  safeDistance *= scaleFactor;
  distanceFromWall *= scaleFactor;

  //Apply the scalefactor to the position of each of the sensors
  //sensorObstacleDist[i] will automatically be less since the map being measured is smaller
  //for (int i = 0; i < numSensors; i++)
  //{
  //  sensorX[i] *= scaleFactor;
  //  sensorY[i] *= scaleFactor;
  //}

  //Applies scale factor to sensors on the robot
  //---should probably be moved to the robot or sensor display function
  for (int k = 0; k < myRobot.sensors.size(); k++)
  {
    myRobot.sensors.get(k).sensorXPos *= scaleFactor;
    myRobot.sensors.get(k).sensorYPos *= scaleFactor;
    myRobot.sensors.get(k).sensorMaxDetect *= scaleFactor;
  }
  
  //Applies scale factor to each particle
  for (int k = 0; k < maxParticles; k++)
  {
    for (int i = 0; i < numSensors2; i++)
    {
      particles[k].sensors.get(i).sensorXPos *= scaleFactor;
      particles[k].sensors.get(i).sensorYPos *= scaleFactor;
      particles[k].sensors.get(i).sensorMaxDetect *= scaleFactor;
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//PVector calcVectorGoToGoal()
//{
//  PVector result = new PVector();
//  result.x = nextWaypoint.x - myRobot.location.x;
//  result.y = nextWaypoint.y - myRobot.location.y;  
//  return result; //.normalize();
//}

/////////////////////////////////////////////////////////////////////////////////////////////////
//PVector calculateVectorBlendedAOGTG()
//{
//  PVector result = new PVector();
//  float dist = vectorAvoidObstacles.mag();
//  float beta = 0.002;          //The smaller this value gets the smaller sigma becomes
//  float sigma = 1 - exp(-beta*dist);
//  PVector gtgBlend = new PVector();
//  PVector aoBlend = new PVector();  
  
//  PVector.mult(vectorGoToGoal,sigma, gtgBlend);
//  PVector.mult(vectorAvoidObstacles, (1-sigma), aoBlend); 

//  result = PVector.add(gtgBlend, aoBlend);
//  return result;
//}