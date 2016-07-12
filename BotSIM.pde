//All distances are measured and listed in cm's unless specified otherwise

import processing.opengl.*;
import org.openkinect.freenect.*;
import org.openkinect.processing.*;

// Kinect Library object
Kinect kinect;

float maxKinectDetectNormal = 400.0;  //Maximum distance we are going to use the kinect to detect distance, measured in cm's
float maxKinectDetectTooFar = 800.0;
float maxKinectDeadZone = 40.0;
float maxKinectPersistantView = 200.0;    //All kinect obstacles up to this distance will be persistant
float kinectFOW = 60.0;        //Field of View on the horizontal axis
int skip=10;          //constant used to set the subsample factor fo the kinect data
// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];
PVector kinectPos = new PVector(-20.0, 0.0 ,0.0);    //Position of Kinect sensors on robot. Robot x and y pos is 0,0
float deltaX = tan(radians(kinectFOW/2)) * maxKinectPersistantView;
PVector leftPoint = new PVector(kinectPos.x + maxKinectPersistantView, deltaX, 0);
PVector rightPoint = new PVector(kinectPos.x + maxKinectPersistantView, -deltaX, 0);





PImage img;

boolean followPath = true;    //Setting to control if path must be followd or is it a true bug goal locate algorithm

//Actual distance of measured on ground, measured in cm's, where one pixel = 1cm???
float worldMapScaleX = 800; //3737;      //To be used as the actual distance of the world map x axis, measured in cm
float worldMapScaleY = 800; //1137;

float worldWidthReal = worldMapScaleX;    //New variable that should replace worldMapScaleX
float worldHeightReal = worldMapScaleY;    //New variable for world height that should replace woldMapScaleY

float screenSizeX = 800;
float screenSizeY = screenSizeX * (worldMapScaleY/worldMapScaleX);  //Scale the y size according to ratio between worldMapScaleX an Y

float scaleFactor = screenSizeX / worldMapScaleX;

int viewPortWidth = 800;    //Area that will be displayed on the screen using the same units as worldWidthReal
int viewPortHeight = 800;

int graphicBoxWidth = 800;    //Pixel size of actual screen real estate which will display the viewPort data
int graphicBoxHeight = 800;

int vpX = 0;      //The x-coord of the top left corner of the viewPort
int vpY = 800;      ///The y-coord of the top left corner of the viewPort



boolean wallDetect = false;

Robot myRobot;          //Creat a myRobot instance
float diameter = 45.0;

//Negative value for robotStart.z (heading) creates a left turn, positive values creates a right turn
PVector robotStart = new PVector (worldMapScaleX/3 , worldMapScaleY/2 , 0.0);    //Position of robot in map on the screen

final int maxParticles = 00;
Robot[] particles = new Robot[maxParticles];
final float noiseForward = 1.0;            //global Noisevalues used to set the noise values in the praticles
final float noiseTurn = 0.1;
final float noiseSense = 5.0;

float moveSpeed = 0;                    //Globals used to define speeds and turn angle
float moveAngle = 0;

float turnGain = 0.1;
float moveGain = 0.01;
float blendGain = 0.5;      //Gain used when blending the AO and GTG vectors;
float normaliseGain = 100.0;

float safeZone = 20.0;          //Safe area around target assumed the robot reached its goal;

//safeDistance cannot be less than minDetectDistance
int safeDistance = 50;      //If sensor measured distance is less than this value, the robot is too close to an obstacle
float distanceFromWall = 50.0;    //Distance that must be maintained when following the wall


//This section must be removed when only the sensor class is used
float[] sensorX =   {0.0, cos(PI/8*3)* diameter/2, cos(PI/8*2)*diameter/2, cos(PI/8)*diameter/2, diameter/2, cos(PI/8)*diameter/2, cos(PI/4)*diameter/2, cos(PI/8*3)*diameter/2, 0.0};      //Array containing all the sensors X values in the robot frame
float[] sensorY =   {-(diameter/2), -sin(PI/8*3)* diameter/2, -sin(PI/8*2)*diameter/2, -sin(PI/8)*diameter/2, 0.0, sin(PI/8)*diameter/2, sin(PI/4)*diameter/2, sin(PI/8*3)*diameter/2, diameter/2};
float[] sensorPhi = {-PI/2, -PI/8*3, -PI/8*2, -PI/8, 0.0, PI/8, PI/4, PI/8*3, PI/2};
float[] sensorGains = {1.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 1.0};    //Gains used to indicate imprtance of sensor values
int numSensors = sensorX.length;    //Determines the amount of sensor elements present
float[] sensorObstacleDist = new float[numSensors];

float[] vectorAO_GTG = {0.0, 0.0};    //x and y values for avoid obstacle and go-to-goal combined vector
float[] vectorAO = {0.0, 0.0};      //x and y values for avoid obstacle vector
float[] vectorGTG = {0.0, 0.0};      //x and y values for vector go-to-goal
float goalX = screenSizeX / 2;            //Goal's X and Y coordinates, set up by clicking with the mouse on the screen
float goalY = screenSizeY / 2;
//This section must be removed when only the sensor class is used

PVector vectorAOGTG = new PVector();
PVector vectorAvoidObstacles = new PVector();
PVector coordsAvoidObstacles = new PVector();    //Coords on the world frame, holding the point of the avoid obstacle vector
PVector vectorGoToGoal = new PVector();
PVector vectorBlendedAOGTG = new PVector();      //Holds the vector which is blended between AvoidObstacles an GoToGoal
PVector nextWaypoint = new PVector();
PVector vectorAOFWD = new PVector();

float[] vectorWall = {0.0, 0.0};      //x and y values representing the vector of a piece of wall for follow wall procedure
float[] vectorWallDist = {0.0, 0.0};  //x and y values for a line perpendicular to the wall vector
float[] vectorAwayFromWall = {0.0, 0.0};  //x and y values for vector pointing away from the wall
float[] vectorFollowWall = {0.0, 0.0};    //Vector pointing in the direction the robot must move when following the wall




int numSensors2 = 7;          //Number of sensors used by the new code

int minDetectDistance = 10;        //Closer than this value and the sensors do not return valid data
float maxDetectDistance = 200.0;


PVector goalXY = new PVector(screenSizeX / 2, screenSizeY / 2);       //Holds the goal's x and y coords
float startX = 0;          //Starting point for straight line to goal used by Bug algorithm families
float startY = 0;
float x_vector_avoid = 0.0;
float y_vector_avoid = 0.0;
float phi_avoid = 0.0;
float errorAngle = 0.0;

float[] closest1 = {0.0, 0.0};
float[] closest2 = {0.0, 0.0};

int stateVal = 0;      //Values used to indicate which state the robot is currently in

boolean showVal = false;
boolean step = true;

//Measurement of tiles to be used for occupancy grid in cm's scaled to represented size in real world
int tileSize = 25;
int maxTilesX = 0;
int maxTilesY = 0;
Tile tile[][];

int oldMillis, newMillis, time, old_time;
int delta_t = 500;

void setup()
{
  kinect = new Kinect(this);
  kinect.initDepth();
  
  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) 
  {
    depthLookUp[i] = rawDepthToMeters(i);
  } 
  
//-------------------------------------------------------------------------------
  //Initialise Robot
  myRobot = new Robot("ROBOT", diameter);        //Create a new robot object
  myRobot.set(robotStart.x, robotStart.y, robotStart.z);

  //Add sensors to the robot object
  for (int k=0; k<numSensors2; k++)
  {
    myRobot.addSensor(0, 0, -PI/2 + PI/(numSensors2-1)*k);
    
    //myRobot.addSensor(0,0,0);
    myRobot.sensors.get(k).sensorMinDetect = minDetectDistance;
  }
  
  
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  
  
  
  //img = loadImage("blank.png");         //Loads image
  //img = loadImage("Floorplan.png");
  img = loadImage("kamer3.png");         //Loads image
  img.resize(int(screenSizeX), int(screenSizeY));
  
  tileSize *= scaleFactor;        //Aplies scale factor to the tile size
    
  maxTilesX = ceil((float(img.width)/float(tileSize)));
  maxTilesY = ceil((float(img.height)/float(tileSize)));  
  
  println("img.width : "+img.width+", img.Height: "+img.height);
  println(maxTilesX+","+maxTilesY);
  
  tile = new Tile[int(maxTilesX)][int(maxTilesY)];
  
  //Sets up a 2D array which will hold the world Tiles
  for (int x = 0; x < maxTilesX; x++) //<>//
  {
    for (int y = 0; y < maxTilesY; y++)
    {
      tile[x][y] = new Tile(int(x*tileSize + tileSize/2), int(y*tileSize + tileSize/2));
      //tile[x][y] = new Tile(toWorldX(int(x*tileSize + tileSize/2)), toWorldY(int(y*tileSize + tileSize/2)));
    }
  }
  
  //Scans the pixels of the background image to build the occupancy grid
  img.filter(THRESHOLD);              //Convert image to greyscale
  for (int x = 0; x < screenSizeX; x++)
  {
    for (int y = 0; y < screenSizeY; y++)
    {
      color c = img.get(x,y);
      if (c == color(0))
      { 
        tile[toWorldX(x)/tileSize][toWorldY(y)/tileSize].gravity = 1;
        tile[toWorldX(x)/tileSize][toWorldY(y)/tileSize].tileType = "MAP";      //Set tileType to PERMANENT/MAP OBSTACLE
        tile[toWorldX(x)/tileSize][toWorldY(y)/tileSize].update();
      }      
    }
  } 

  
  

  
  //-------------------------------------------------------------------------------
  //Create particles to localise robot
  for (int i = 0; i < maxParticles; i++)
  {
    particles[i] = new Robot("PARTICLE");
    particles[i].set(screenSizeX/2, screenSizeY/2, -PI/2);
    particles[i].setNoise(noiseForward, noiseTurn, noiseSense);    //Add noise to newly created particle

    for (int k = 0; k < numSensors2; k++)
    {
      particles[i].addSensor(0, 0, -PI/2 + PI/(numSensors2-1)*k);
    }
  }
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  applyScale();    //Applies the scale to all physical quantities

  //size(100,100,OPENGL);
  surface.setResizable(true);
  surface.setSize(int(screenSizeX), int(screenSizeY));

  //Change particle x and y values to prevent them from being inside walls
  //for (int i=0; i < maxParticles; i++)
  //{
  //  color col = img.get (int(particles[i].x) ,int(particles[i].y));    //Test pixel colour to determine if there is an obstacle
  //  if (red(col) == 0)
  //  {
  //    while(red(col) == 0)
  //    {
  //      particles[i].x = random (0, screenSizeX);
  //      particles[i].y = random (0, screenSizeY);
  //      col = img.get (int(particles[i].x) ,int(particles[i].y));    //Test pixel colour to determine if there is an obstacle
  //    }
  //  }
  //}
  
  //allNodes.clear();
  //doQuadTree(0,0, maxTilesX, maxTilesY, QuadTreeLevel);
  //allNodes.add( new Node(myRobot.location.x, myRobot.location.y, "START", allNodes.size()));
  //allNodes.add( new Node(goalXY.x, goalXY.y, "GOAL", allNodes.size()));
     
  ////println("\nNumber of allNodes: "+allNodes.size());        //Print the total number of nodes
  //nodeLink();
  //findPath();
  
  printArray(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 115200);  
  delay(5000);      //Delay to make sure the Arduino initilaises before data is sent
  myPort.write("<v00\r");    //Sends a velcoity of 0 to the chassis
  delay(500);
  myPort.write("<w0\r");      //sends a turn rate of 0 to the chassis
  delay(500);  
  
  myPort.clear();
  // Throw out the first reading, in case we started reading 
  // in the middle of a string from the sender.
  inData = myPort.readStringUntil(lf);
  inData = null;
  myPort.bufferUntil(lf);        //Buffers serial data until Line Feed is detected and then only reads serial data out of buffer  
}








void draw()
{ 
  surface.setTitle(int(frameRate)+" fps");        //Add framerate into title bar
  
  if (showVal)
  {
   for (int k=0; k<numSensors2; k++) print(int(myRobot.sensors.get(k).sensorObstacleDist)+"\t");
   println("\nState: "+stateVal+", CollisionFlag: "+myRobot.collisionFlag);
   println();

   for (int k = 0; k< maxParticles; k++)
   {
     for (int i = 0; i < numSensors2; i++)
     {
       print (int(particles[k].sensors.get(i).sensorObstacleDist)+"\t");
     }
     print ("PROB: "+ particles[k].prob);
     println();
   }
   println();
   showVal = false;
  }

  
  
  //parseSerialData();

  if (step)
  {
    background (img);        //draws map as background
    drawTiles();   
    drawTarget();
    myRobot.display();
    
    //isInFOW();    
    //drawPixels();      //Draws the data from the Kinect sensors on the screen    
    
    //oldMillis = newMillis;
    //newMillis = millis();
    //textSize(16);  
    //textAlign(LEFT, TOP);
    //fill(0);
    //text("frame rate (ms): "+(newMillis - oldMillis),5,5);
   
    allNodes.clear();
    //###Quadtree values must be changed form 0,0 to world's min x and y values else negative coords 
    //###  will not be used in path planning
    doQuadTree(0,0, maxTilesX, maxTilesY, QuadTreeLevel); //<>//
    allNodes.add( new Node(myRobot.location.x, myRobot.location.y, "START", allNodes.size()));
    allNodes.add( new Node(goalXY.x, goalXY.y, "GOAL", allNodes.size()));  
    
    //oldMillis = millis();
    ////nodeLink();
    //time = millis() - oldMillis;
    //println("Node Link time: "+time);
  
    //findPath();
   
    //PlotRobot();
    //calcProgressPoint();
    
    //Draws an ellipse at the centerpoint of the kinect's position on the robot
    //PVector returnVal = transRot(myRobot.location.x, myRobot.location.y, myRobot.heading, kinectPos.x, kinectPos.y);
    //fill(255,255,0);
    //ellipse(returnVal.x, returnVal.y, 10,10);  
    
    //Displays the node position on the map
    //for (Node n: allNodes)
    //{
    //   n.display();     
    //}
    
    //int startTime = millis();
    //myRobot.sense();          //Makes use of sensor class to detect obstacles
    
    //for (int k = 0; k < maxParticles; k++)
    //{
    //  particles[k].sense();
    //  particles[k].measureProb();
    //}
    
    //int endTime = millis();
    //println(endTime - startTime);
    
    //if (stateVal != 0)
    //{
    //  updateParticles();
    //  resample();
    //}
  
  
    step = true;

    //Calculates the vector to avoid all obstacles
    //vectorAvoidObstacles = calcVectorAvoidObstacles();
    
    //Calculates the Go To Goal vector
    //vectorGoToGoal.x = nextWaypoint.x - myRobot.location.x;
    //vectorGoToGoal.y = nextWaypoint.y - myRobot.location.y;
    
    ////Calculates the vector which blends the Go to Goal and Avoid Obstacles
    //vectorAOFWD = PVector.add(vectorGoToGoal, vectorAvoidObstacles);  
    
    ////Calcualtes the angle in which the robot needs to travel   
    //float angleToGoal = atan2(vectorAOFWD.y,vectorAOFWD.x) - myRobot.heading;
        
    //if (angleToGoal < (-PI)) angleToGoal += 2*PI;
    //if (angleToGoal > (PI)) angleToGoal -= 2*PI;
    
    //fill(0);
    //text (angleToGoal, 55,700);
       
    ////Caclualtes the distance between robot and goal to determine speed    
    //float velocityToGoal = dist (nextWaypoint.x, nextWaypoint.y, myRobot.location.x, myRobot.location.y) /5;
    
    //Routine used to poll driverlayer every delta_t millis in order to get sensor and position data
    time = millis();  
    int interval = time - old_time;
    if (interval > delta_t)
    {
      //println("velocity: "+velocityToGoal+ ", angle: " + angleToGoal);
      //updateRobot(velocityToGoal, angleToGoal);
      old_time = time;
    }
  }
  //dispVectors();      //Displays different vectors, ie: Go-To-Goal, Avoid Obstacle, etc  
}

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

//Draws the world using tiles
void drawTiles()
{
  for (int x = 0; x < maxTilesX; x++)
  {
    for (int y = 0; y < maxTilesY; y++)
    {
      tile[x][y].tileDraw();
      //stroke(150);        //Lines between tiles are black
      //strokeWeight(1);  //Stroke weight makes the lines very light
      //fill(tile[x][y].gravityCol,200);
      
      //rectMode(CENTER);      //Use the first two coords as centerpoint and next two as width and height of rectangle
      //rect(tile[x][y].tilePos.x, tile[x][y].tilePos.y, tileSize, tileSize);
      ////rect(toScreenX(int(x*tileSize)), toScreenY(int(y*tileSize)), tileSize, tileSize);  //Draws a rectangle to indicate the tile     
      

      tile[x][y].drawTileForce(); 
      
      //textAlign(CENTER,BOTTOM);
      //textSize(10);
      //fill(0);
      
      //text(x+":"+y, tile[x][y].tilePos.x, tile[x][y].tilePos.y);
      
      ////text(int(tile[x][y].tilePos.x), tile[x][y].tilePos.x, tile[x][y].tilePos.y);
      ////textAlign(CENTER,TOP);
      ////text(int(tile[x][y].tilePos.y), tile[x][y].tilePos.x, tile[x][y].tilePos.y);

      
      tile[x][y].update();
    }
  }
}

//###############################################################################################
//Checks to see if tile center point is inside kienct Field of View but closer than the maximum Peristant view value.
//(http://stackoverflow.com/questions/13300904/determine-whether-point-lies-inside-triangle)
//Kinect data outside of this area will not influence the map.
//Kinect data inside this area will be overwritten if in the field of view.
//The purpose is to collect obstacle data in order to 'remeber where obstacles are when the kinect moves and these obstacle go into the deadzone

void isInFOW()
{
  float alpha = 0.0;
  float beta = 0.0;
  float gamma =0.0;
  PVector newKinectPos = transRot(myRobot.location.x, myRobot.location.y, myRobot.heading, kinectPos.x, kinectPos.y);
  PVector newLeftPoint = transRot(myRobot.location.x, myRobot.location.y, myRobot.heading, leftPoint.x, leftPoint.y);
  PVector newRightPoint = transRot(myRobot.location.x, myRobot.location.y, myRobot.heading, rightPoint.x, rightPoint.y);
  
  line (newKinectPos.x, newKinectPos.y, newLeftPoint.x, newLeftPoint.y);
  line (newKinectPos.x, newKinectPos.y, newRightPoint.x, newRightPoint.y);
  
  for(int y = 0; y < maxTilesY; y++)
  {
    for(int x = 0; x < maxTilesX; x++)    
    {      
      if (tile[x][y].tileType == "UNASSIGNED" || tile[x][y].tileType == "KINECT")
      {
        alpha = ((newLeftPoint.y - newRightPoint.y)*(tile[x][y].tilePos.x - newRightPoint.x) + (newRightPoint.x - newLeftPoint.x)*(tile[x][y].tilePos.y - newRightPoint.y)) / 
                      ((newLeftPoint.y - newRightPoint.y)*(newKinectPos.x - newRightPoint.x) + (newRightPoint.x - newLeftPoint.x)*(newKinectPos.y - newRightPoint.y));
        beta = ((newRightPoint.y - newKinectPos.y)*(tile[x][y].tilePos.x - newRightPoint.x) + (newKinectPos.x - newRightPoint.x)*(tile[x][y].tilePos.y - newRightPoint.y)) / 
                     ((newLeftPoint.y - newRightPoint.y)*(newKinectPos.x - newRightPoint.x) + (newRightPoint.x - newLeftPoint.x)*(newKinectPos.y - newRightPoint.y));
        gamma = 1.0f - alpha - beta;
        
        if ((alpha > 0 & beta > 0 & gamma > 0))
        {
          tile[x][y].tileType = "UNASSIGNED";
        }        
      }
    }
  }
}

//###############################################################################################
//Updates each particle accoridng to robot movement
void updateParticles()
{
  //Update particle movement
  for (int i = 0; i < maxParticles; i++)
  {
    particles[i].move(moveAngle, moveSpeed);
  }

  //Display updated particles
  for (int i=0; i < maxParticles; i++)
  {
    particles[i].display();
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
  myRobot.maxTurnRate *= scaleFactor;
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
//###############################################################################################
//Main FSM for robot movement and decisions
void PlotRobot()
{  
  float distanceToTarget = PVector.dist(goalXY, myRobot.location);

  float phi_GTG = calcGoalAngle(vectorGoToGoal.x, vectorGoToGoal.y);
  float phi_AO = calcGoalAngle(vectorAvoidObstacles.x, vectorAvoidObstacles.y);
  float phi_AO_GTG = calcGoalAngle(vectorAO_GTG[0], vectorAO_GTG[1]);
  float phi_FW = calcGoalAngle(vectorFollowWall[0], vectorFollowWall[1]);
  
  switch (stateVal)
  {
  case 0:    //Stop State / Arrived at Goal
    if (distanceToTarget > safeZone)
    {
      stateVal = 1;
    } else
    {
      distanceToTarget = 0;
    }
    break;

  case 1:    //Go straight to goal
    //calcErrorAngle(phi_GTG);

    if (distanceToTarget <= safeZone)   //Robot is close enough to goal, stop robot
    {
      stateVal = 0;
    }

    if (myRobot.collisionFlag)
    {
      stateVal = 2;
    }

    if (!myRobot.collisionFlag)
    { 
      if (followPath)
      {
        //finalPath is always from the current robot position to the goal. The last element is the robot's current position
        //  the 2nd last element is the next waypoint
        int nextWP = finalPath.get(finalPath.size()-2); //index of next waypont in finalPath array
        nextWaypoint.x = allNodes.get(nextWP).nodeXPos;
        nextWaypoint.y = allNodes.get(nextWP).nodeYPos;
        
        stroke(0,0,255);
        strokeWeight(0);
        fill(0,0,255);
        ellipse (nextWaypoint.x, nextWaypoint.y, 20,20);      
        //phi_GTG = calcGoalAngle(nextWayPointX - myRobot.location.x, nextWayPointY - myRobot.location.y);
      }
      //calcErrorAngle(phi_GTG);
    }

    if ((!myRobot.makingProgress) && (myRobot.collisionFlag))
    {
      stateVal = 2;
    }
    break;

  case 2:    //Avoid obstacle state
    calcErrorAngle(phi_AO);    

    if (myRobot.collisionFlag)
    {
      stateVal = 1;
      myRobot.collisionFlag = false;
    }
    break;

  case 3:
    calcErrorAngle(phi_FW);
    //makingProgress = true;
    if (myRobot.makingProgress) stateVal = 1;
    break;
  }

  moveAngle = min (myRobot.maxTurnRate, (turnGain * errorAngle));  //P controller to turn towards goal
  moveSpeed = min (myRobot.maxSpeed, (moveGain * (distanceToTarget)));  
  
  //myRobot.move(moveAngle, moveSpeed);
  myRobot.display();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//  Calculates a progress point towards the goal.
//  If the robot's distance-to-goal is less than the progress point distance-to-goal,
//      the robot progresses, else do Wall Following

void calcProgressPoint()
{
  // float oldDist = sqrt(pow(goalX - progressPoint[0], 2) + pow(goalY - progressPoint[1], 2));    //Calculates the straight line distance to the goal
  // float newDist = sqrt(pow(goalX - myRobot.state[0], 2) + pow(goalY - myRobot.state[1], 2));
  float oldDist = PVector.dist(goalXY, myRobot.progressPoint);
  float newDist = PVector.dist(goalXY, myRobot.location);
  
  if (newDist <= oldDist)
  {
    myRobot.progressPoint = myRobot.location.copy();    
    myRobot.makingProgress = true;  
  } else
  {
    myRobot.makingProgress = false;    
  }

  strokeWeight(1);
  stroke(0);  
  fill (0, 255, 255);
  ellipse(myRobot.progressPoint.x, myRobot.progressPoint.y, 5, 5);
  strokeWeight(1);
  stroke(0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// This function takes the angle towards the destination point and calclates the folowing:
//    ??1) Converts it from an atan2 angle into a real world angle
//    2) Calculates the difference between the robot's heading and the goal angle
//    3) Converts this difference into the robot's local frame in order to determine left and right turns
// Based on Games Programming: Methods and How to's - Dr James Jordaan Revision 4.1 p196
void calcErrorAngle (float goalAngle)
{
  //if (goalAngle < 0) goalAngle += (2*PI);        //Change goal angle from atan2 to radians (global frame)
  errorAngle = goalAngle - myRobot.heading;
  if (errorAngle < -PI) errorAngle += (2*PI);
  if (errorAngle > PI) errorAngle -= (2*PI);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//This function will calculate the flow field using the MAP and USER tiles
PVector calcVectorAvoidObstacles()
{  
  PVector vectorAO = new PVector();
  
  for(int y = 0; y < maxTilesY; y++)
  {
    for(int x = 0; x < maxTilesX; x++)    
    {      
      if (tile[x][y].tileType == "MAP" || tile[x][y].tileType == "USER")
      {
        if (tile[x][y]. gravity != 0)
        {
          vectorAO.add(tile[x][y].field);
        }
      }
    }
  }
  
  //for (int k = 0; k < myRobot.sensors.size(); k++)
  //{
  //  //TransRotate sensor distance value to robot frame
  //  tempCoords = transRot(myRobot.sensors.get(k).sensorXPos, myRobot.sensors.get(k).sensorYPos, myRobot.sensors.get(k).sensorHAngle, myRobot.sensors.get(k).sensorObstacleDist, 0);
    
  //  //Add all the x's and y's together to get combined vector of avoid obstacles        
  //  vectorAO.add(tempCoords);
  //}
  ////vectorAO.normalize();
  ////println(vectorAO.mag());
  ////Transrotate avoidObstacles coords into world frame with a scaling factor
  //tempCoords = transRot(myRobot.location.x, myRobot.location.y, myRobot.heading, vectorAO.x, vectorAO.y);
  
  //tempCoords.x = tempCoords.x - myRobot.location.x;
  //tempCoords.y = tempCoords.y - myRobot.location.y;
  
  return vectorAO;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

float calcGoalAngle(float vectX, float vectY)
{
  float angle = atan2 (vectY, vectX);
  //if (angle <= 0) angle += (2*PI);
  return angle;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void drawTarget()
{
  stroke(0);
  fill(255, 0, 0);
  strokeWeight(1);
  ellipse (toScreenX(int(goalXY.x)), toScreenY(int(goalXY.y)), safeZone*3, safeZone*3);
  stroke(0);
  fill(255);
  ellipse (toScreenX(int(goalXY.x)), toScreenY(int(goalXY.y)), safeZone*2, safeZone*2);
  stroke(0);
  fill(0);
  ellipse (toScreenX(int(goalXY.x)), toScreenY(int(goalXY.y)), safeZone, safeZone);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void dispVectors()
{  
  //Draws a vector pointing away from all the obstacles
  strokeWeight(4);
  stroke(255,0,0);
  line(myRobot.location.x, myRobot.location.y, myRobot.location.x + vectorAvoidObstacles.x*10, myRobot.location.y + vectorAvoidObstacles.y*10);
  
  strokeWeight(5);
  stroke(0,255, 0);
  
  line(myRobot.location.x, myRobot.location.y, myRobot.location.x + vectorGoToGoal.x, myRobot.location.y + vectorGoToGoal.y);
  
  strokeWeight(5);
  stroke(0,0, 255);
  //line(myRobot.location.x, myRobot.location.y, myRobot.location.x + vectorBlendedAOGTG.x * 100, myRobot.location.y + vectorBlendedAOGTG.y * 100);
  line(myRobot.location.x, myRobot.location.y, myRobot.location.x + vectorAOFWD.x * 10, myRobot.location.y + vectorAOFWD.y * 10);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//Rotates and translates an X and Y coordinate onto a local frame using the local frame's X,Y and HEADING
//Returns a PVector with new x and y coordinates
PVector transRot (float x_frame, float y_frame, float phi_frame, float x_point, float y_point)
{
  float x_temp = cos(phi_frame) * x_point - sin(phi_frame)*y_point + x_frame; //Uses transformation and rotation to plot sensor gloablly
  float y_temp = sin(phi_frame) * x_point + cos(phi_frame)*y_point + y_frame;
  PVector result = new PVector(x_temp, y_temp);
  return result;
}
//==========================================================================
//
void mousePressed()
{
  if (mousePressed && (mouseButton == LEFT)) changeGoal();
  
  if (mousePressed && (mouseButton == RIGHT))
  { 
    //robotStart.x = toWorldX(int(mouseX));
    //robotStart.y = toWorldY(int(mouseY));
    
    myRobot.location.x = toWorldX(int(mouseX));
    myRobot.location.y = toWorldY(int(mouseY));

    //Resets progress point when target is moved to the current mouse position    
    myRobot.progressPoint.x = toWorldX(int(mouseX));
    myRobot.progressPoint.y = toWorldY(int(mouseY));
    myRobot.makingProgress = true;
  }
}

//Change the goal location everytime the mouse is clicked
void changeGoal()
{  
  goalXY.x = toWorldX(int(mouseX));
  goalXY.y = toWorldY(int(mouseY));

  startX = myRobot.location.x;
  startY = myRobot.location.y;

  stateVal = 1;
  
  //Resets progress point when target is moved to the current robot position  
  myRobot.progressPoint = myRobot.location;
  myRobot.makingProgress  = true;
  
  //Changes the GOAL node to new goal position
  for (int k = 0; k < allNodes.size(); k++)
  {
    if (allNodes.get(k).nodeType == "GOAL")
    {
      allNodes.get(k).nodeXPos = goalXY.x;
      allNodes.get(k).nodeYPos = goalXY.y;        
    }
  }
  //Link all the nodes together again
  nodeLink();
  //Calculate new shortest route to GOAL
  findPath();
}

void keyPressed()
{
  if (key == ' ') showVal = true;

  if (key =='s') requestSerialPosition();

  //Use this key to enable or disable obstacle
  if (key == 'o')
  {
    int worldMouseX = toWorldX(mouseX)/tileSize;
    int worldMouseY = toWorldY(mouseY)/tileSize;
    switch(tile[worldMouseX][worldMouseY].tileType)
    {
      case "UNASSIGNED":
      {            
        tile[worldMouseX][worldMouseY].tileType = "USER"; //Set tileType to USER obstacle
        //tile[int(mouseX/tileSize)][int(mouseY/tileSize)].tileType = "USER"; //Set tileType to USER obstacle        
        tile[worldMouseX][worldMouseY].update();
        break;
      }
      
      case "USER":
      {        
        tile[worldMouseX][worldMouseY].tileType = "UNASSIGNED"; //Set tileType to UNASSIGNED obstacle        
        tile[worldMouseX][worldMouseY].update();
        break;
      }
    }
  }
}



//Implementation of the following website
//http://www.libertybasicuniversity.com/lbnews/nl112/mapcoor.htm

//Creates a viewport which is used to view only a specific area of the world map
//It also plots world coordinates onto the screen i.e: Inverting the y-axis
int toScreenX(int _x)
{
  return ((graphicBoxWidth / viewPortWidth) * (_x - vpX));
}

int toScreenY(int _y)
{
  return ((graphicBoxHeight / viewPortHeight) * (vpY - _y));
}


int toWorldX (int _x)
{
  return int((float(_x) / float(graphicBoxWidth) * float(viewPortWidth) + vpX));  
}

int toWorldY (int _y)
{
  return int((vpY - float(_y) / float(graphicBoxHeight) * float(viewPortHeight))-1);
}