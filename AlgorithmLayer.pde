//## Use the following website to install Kinect drivers for Windows
//## https://github.com/shiffman/OpenKinect-for-Processing

//## Use the following website and follow instructions to install Kinect libraries for Processing
//## http://shiffman.net/p5/kinect/


//Below is a list of comment types
//    : Old comment type, must be rewritten using new style
//##  : Comments explaining pieces of code. These comments wont change a lot
//!!  : Fixes that need to be made to commented pieces of code

//##When true the robot does not access any serial ports and uses location and sensor data from the simulation
boolean simMode = false;
boolean showVal = false;
boolean step = true;
boolean nextStep = false;
boolean followPath = true;    //Setting to control if path must be followd or is it a true bug goal locate algorithm
float timeScale = 0.1;  //value used to make the simulator slower or faster

//### Inital position of robot in the world map where 0,0 is the left bottom corner
//  Ultimately the robot will not initially know where it is. These values can be used to plot the robot somewhere in the world map before 
//    localisation moves the robot sprite to its localised location
PVector robotPosOffset = new PVector (300, 441, random(-PI,PI));
PVector goalXY = new PVector(300, 200);       //Holds the goal's x and y coords
//PVector goalXY = new PVector(imgWidth * 0.5, imgHeight/2);       //Holds the goal's x and y coords

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
float kinectTilt = 0.0;        //Tilt angle of kinect sensor
int skip=10;          //constant used to set the subsample factor fo the kinect data
// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];
PVector kinectPos = new PVector(-20.0, 0.0 ,0.0);    //Position of Kinect sensors on robot. Robot x and y pos is 0,0
float deltaX = tan(radians(kinectFOW/2)) * maxKinectPersistantView;
PVector leftPoint = new PVector(kinectPos.x + maxKinectPersistantView, deltaX, 0);
PVector rightPoint = new PVector(kinectPos.x + maxKinectPersistantView, -deltaX, 0);

float alpha = 0.1;  //### Scaling value of attractive force - must be moved out of Tile class
float s = 100.0;    //### Spread of the goal's circle of influnce - must be moved out of Tile class
float obstacleS = 50.0; //## Obstacle circle of influence
float beta = 0.3; //scale value of pushing force
float infinity = 1000.0;

PImage img;

//Actual distance of measured on ground, measured in cm's
float worldMapScaleX = 0; //3737;      //To be used as the actual distance of the world map x axis, measured in cm
float worldMapScaleY = 0; //1137;

//float worldWidth = worldMapScaleX;    //New variable that should replace worldMapScaleX
//float worldHeight = worldMapScaleY;    //New variable for world height that should replace woldMapScaleY

//Select the map to be used and set the imgHeight and imgWidth values to the x and y size of the graphic
//String mapName = "Floorplan.png";
//String mapName = "blank.png"; float worldWidth = 780; float worldHeight = 780;   //The actual dimensions in the real world represented by this map
//String mapName = "Huisplan.png";
String mapName = "kamer3.png"; float worldWidth = 780; float worldHeight = 780;
//String mapName = "BibMapPNG.png"; float worldWidth = 2390; float worldHeight = 2390;   //The actual dimensions in the real world represented by this map
//String mapName = "Bib Map2.png"; float worldWidth = 2718; float worldHeight = 2390;   //The actual dimensions in the real world represented by this map


 


//-- These variables need to be fixed. Remove imgWidth and imgHeight and replace with worldHeight and worldWidth to make more sense
//-- worldWidth and worldHeitgh is the size covered by the image on the real world
float imgWidth = worldWidth; //1130;      //Actual dimensions the image represents in same dimensions as worldWidth and worldHeight
float imgHeight = worldHeight; //530;

//## The viewport is the piece of real world map that will be visible on the screen - If you zoom in the viewport values will decrease
//      therefore showing a smaller piece of the real world map
float viewPortWidth = worldWidth; //1800;    //Area that will be displayed on the screen using the same units as worldWidthReal
float viewPortHeight = worldHeight; //1800;

float graphicBoxWidth = 800;    //Pixel size of actual screen real estate which will display the viewPort data
float graphicBoxHeight = 800;

float screenSizeX = graphicBoxWidth;
float screenSizeY = graphicBoxHeight; //screenSizeX * (worldMapScaleY/worldMapScaleX);  //Scale the y size according to ratio between worldMapScaleX an Y

float vpX = 0.0; //The x-coord of the top left corner of the viewPort
float vpY = worldHeight; //The y-coord of the top left corner of the viewPort

float scaleFactor = 0.0;

boolean wallDetect = false;

Robot myRobot;          //Creat a myRobot instance
float diameter = 45.0;



final int maxParticles = 1000;
Robot[] particles = new Robot[maxParticles];
final float noiseForward = 1.0;            //global Noisevalues used to set the noise values in the praticles
final float noiseTurn = 0.1;
final float noiseSense = 5.0;

float moveSpeed = 0.0;                    //Globals used to define speeds and turn angle
float moveAngle = 0.0;

float turnGain = 1.0; //0.1;
float moveGain = 1.0; //0.01;
float blendGain = 0.5;      //Gain used when blending the AO and GTG vectors;
float normaliseGain = 100.0;

float safeZone = 10.0;          //Safe area around target assumed the robot reached its goal;

//safeDistance cannot be less than minDetectDistance
float safeDistance = 50.0;      //If sensor measured distance is less than this value, the robot is too close to an obstacle
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
float goalX = 0.0;            //Goal's X and Y coordinates, set up by clicking with the mouse on the screen
float goalY = 0.0;
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

float minDetectDistance = 10.0;        //Closer than this value and the sensors do not return valid data
float maxDetectDistance = 200.0;



float startX = 0;          //Starting point for straight line to goal used by Bug algorithm families
float startY = 0;
float x_vector_avoid = 0.0;
float y_vector_avoid = 0.0;
float phi_avoid = 0.0;
float errorAngle = 0.0;

float[] closest1 = {0.0, 0.0};
float[] closest2 = {0.0, 0.0};

int stateVal = 0;      //Values used to indicate which state the robot is currently in



//Measurement of tiles to be used for occupancy grid in cm's scaled to represented size in real world
float tileSize = 25;
int maxTilesX = 0;
int maxTilesY = 0;
Tile tile[][];

int oldMillis, newMillis, time, old_time;
int delta_t = 500;

boolean allowTX = false;    //Allows data to be transmitted to the driverlayer
boolean allowV = false;      //Allows the v movement of the robot


//##The AGENT is a 'ghost' robot showing what the actual robot should be doing when travelling towards the goal. This is basically
//    just used to test attractive and repulsive forces and does not at this stage make use of path planning
PVector agent = new PVector(robotPosOffset.x, robotPosOffset.y, robotPosOffset.z); //10.0, 10.0, 0.0);

int ts = 12;  //textSize value used to display information on the graphical screen

String frameText;    //## String used to change what is displayed in the simulation frame

float accuDist = 0.0;    //accumulated distance from robot movement
float accuTime = 0.0;      //accumulated time from robot movements

boolean mapChange = true;



void setup()
{
  //###Calculates cale factor used to scale all screen avatars/objects
  scaleFactor = graphicBoxWidth / viewPortWidth;
  //### Set viewpoint x and y so that robot is in the middle of the viewPort with startup
  //vpX = robotPosOffset.x - viewPortWidth / 2.0;
  //vpY = robotPosOffset.y + viewPortHeight / 2.0;
  
  if(!simMode)
  {
    //kinect = new Kinect(this);
    //kinect.initDepth();
    //kinectTilt = kinect.getTilt();
  }
  
  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) 
  {
    depthLookUp[i] = rawDepthToMeters(i);
  } 
  
//-------------------------------------------------------------------------------
  //Initialise Robot
  myRobot = new Robot("ROBOT", diameter);        //Create a new robot object
  myRobot.set(robotPosOffset.x, robotPosOffset.y, robotPosOffset.z);
  //myRobot.set(imgWidth/2,imgHeight/2,robotPosOffset.z);

  //Add sensors to the robot object
  for (int k=0; k<numSensors2; k++)
  {
    //### Add numSensors2 amount of sensors all with x,y loacation = 0,0
    myRobot.addSensor(0, 0, -PI/2 + PI/(numSensors2-1)*k);
    myRobot.sensors.get(k).sensorMinDetect = minDetectDistance;
  }
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  
  
  
  img = loadImage(mapName);         //Loads image
  
  //### Resize image to image width and height represented by the world
  img.resize(int(imgWidth), int(imgHeight));  
  
  //worldWidth = img.width+viewPortWidth/2;
 // worldHeight = img.height+viewPortHeight/2;  
  
  surface.setResizable(true);
  surface.setSize(int(graphicBoxWidth), int(graphicBoxHeight)); 
    
  //##Calculates the number of tiles in X and in Y
  maxTilesX = ceil((float(img.width)/(tileSize)));
  maxTilesY = ceil((float(img.height)/(tileSize)));
  
  //## Calculates the smallest binary value that can be used to calculate the tilesize with
  int xx = 0;
  while (int(pow(2,xx)) < int(maxTilesX))
  { 
    xx++;    
  }   //<>// //<>//
  
  int yy = 0;
  while (int(pow(2,yy)) < int(maxTilesY))
  { 
    yy++;    
  }
  tileSize = img.width/pow(2,xx); 
  
  //##Calculates the number of tiles in X and in Y
  maxTilesX = ceil((float(img.width)/(tileSize)));
  maxTilesY = ceil((float(img.height)/(tileSize)));
  
  //!!!!! Not neccessary since the number of tiles are calculated as a power of 2   
  //### Make sure the amount of tiles is ALWAYS an even number
  //###  Currently the quad tree process divides the number of tiles to create the quads and wont work correctly if maxTiles is odd
  //if (maxTilesX % 2 != 0) maxTilesX++;
  //if (maxTilesY % 2 != 0) maxTilesY++;
  
  println("img.Width : "+img.width+", img.Height: "+img.height);
  //println("worldHeight :"+worldHeight+", worldWidth: "+worldWidth);
  println("scaleFactor :"+scaleFactor);
  println(maxTilesX+","+maxTilesY);
  
  tile = new Tile[maxTilesX][maxTilesY];
  
  //### Calculates strating coords for tiles to ensure the 4 center tiles have their corners touching
  //###  in the middle of the screen 
  //float _startX = img.width/2.0 - tileSize/2.0 - tileSize*(maxTilesX/2.0 - 1); // -(tileSize * (maxTilesX - 1) / 2);
  //float _startY = img.height/2.0 - tileSize/2.0 - tileSize*(maxTilesY/2.0 - 1);
  
  //## Set the starting position of the tiles so tiles will start in the bottom left corner of the map
  float _startX = tileSize/2;
  float _startY = tileSize/2;
  println("startX: "+_startX+", startY: "+_startY);
  
  delay(2000);
  
  //###Sets up a 2D array which will hold the world Tiles
  for (int x = 0; x < maxTilesX; x++)
  {
    for (int y = 0; y < maxTilesY; y++)
    {
      tile[x][y] = new Tile((_startX + tileSize * x), (_startY +  y * tileSize));  
    }
  }
  
  //Scans the pixels of the background image to build the occupancy grid
  img.filter(THRESHOLD, 0.9);              //Convert image to greyscale
  for (int x = 0; x < imgWidth; x++)
  {
    for (int y = 0; y < imgHeight; y++)
    {
      color c = img.get(x,y);      
      if (c == color(0))
      {         
        int tileX = floor((x) / tileSize);
        int tileY = floor((imgHeight - 1 - y) / tileSize);  
        //println(tileX+":"+tileY);
        //print("tileSize: "+tileSize+", x: "+x+"("+toWorldX(x)+"), y: "+y+"("+toWorldY(y)+"), tileX:Y = "+tileX+":"+tileY);
        if ((tileX >= 0) && (tileY >= 0))
        {          
          tile[tileX][tileY].gravity = 1;
          tile[tileX][tileY].tileType = "MAP";      //Set tileType to PERMANENT/MAP OBSTACLE
          tile[tileX][tileY].update();
        }
      }      
    }
  }
  
  
  
  
  //### Calculate new resolution for img resize values to ensure img is displayed accoridng to viewPort width
  float newWidth = imgWidth / viewPortWidth * graphicBoxWidth;
  float newHeight = imgHeight / viewPortHeight * graphicBoxWidth;
  img.resize(int(newWidth), int(newHeight));  
  
  

  
  

  
  //-------------------------------------------------------------------------------
  //Create particles to localise robot
  for (int i = 0; i < maxParticles; i++)
  {
    particles[i] = new Robot("PARTICLE");
    //particles[i].set(robotPosOffset.x, robotPosOffset.y, robotPosOffset.z);  //## Move particle to robot position in order to test probability calculations
    //particles[i].set(robotPosOffset.x + random(-10,10), robotPosOffset.y + random(-10,10), random(-PI,PI));  //## Move particle to robot position in order to test probability calculations
    particles[i].set(random(imgWidth), random(imgHeight), random(-PI,PI));
    particles[i].setNoise(0.05, 0.05, 15.0);    //Add noise to newly created particle

    for (int k = 0; k < numSensors2; k++)
    {
      particles[i].addSensor(0, 0, -PI/2 + PI/(numSensors2-1)*k);
    }
  }
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  //size(100,100,FX2D);
  surface.setResizable(true);
  surface.setSize(int(graphicBoxWidth), int(graphicBoxHeight));

  //Change particle x and y values to prevent particles from being inside walls
  for (int i=0; i < maxParticles; i++)
  { 
    println(i+":"+particles[i].location.x+" "+tileSize+" "+int(particles[i].location.x/tileSize)+" --- "+particles[i].location.y+" "+tileSize+" "+int(particles[i].location.y/tileSize)); 
    while (tile[int(particles[i].location.x / tileSize)][int(particles[i].location.y / tileSize)].tileType != "UNASSIGNED")
    {
      println("BLOCKED");
      particles[i].set(random(imgWidth), random(imgHeight), random(2*PI));      
      println(i+":"+particles[i].location.x+" "+tileSize+" "+int(particles[i].location.x/tileSize)+" --- "+particles[i].location.y+" "+tileSize+" "+int(particles[i].location.y/tileSize));
    }    
  }
  
  
  //##Disable serial port initialisation when in Simulation Mode
  if (!simMode)
  {
    printArray(Serial.list());
    myPort = new Serial(this, Serial.list()[1], 9600);  
    //myPort = new Serial(this, Serial.list()[0], 115200);
    //delay(5000);      //Delay to make sure the Arduino initilaises before data is sent
    //myPort.write("<v00\r");    //Sends a velcoity of 0 to the chassis
    //delay(500);
    //myPort.write("<w0\r");      //sends a turn rate of 0 to the chassis
    //delay(500);  
    
    //myPort.clear();
    // Throw out the first reading, in case we started reading 
    // in the middle of a string from the sender.
    //inData = myPort.readStringUntil(lf);
    //inData = null;    
    myPort.bufferUntil('\r');        //Buffers serial data until Line Feed is detected and then only reads serial data out of buffer
  }
  else
  {
    println("Simulation MODE");
  }
}








void draw()
{     
  if (simMode)
  {
    frameText = int(frameRate)+" fps   -   SIMULATION MODE    -    Time Scale: "+timeScale; 
  }
  else
  {
    frameText = int(frameRate)+" fps";
  }  
  surface.setTitle(frameText);
  
  
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
  
  //##Clears screen with solid background colour
  background (200);            
  
  //##draws map as background
  image(img,toScreenX(0),toScreenY(imgHeight));  
  
  //## Draw tiles used to display obstacle
  drawTiles();   
  
  //##Draw the goal of where the robot needs to go on the screen
  drawTarget();
  
  //##Display text on the screen asociated with keys allocated to doing certain functions
  //displayText();
  
  //## Displays the node positions on the map
  for (Node n: allNodes)
  {
     n.display();     
  }
  
  if (mapChange)
  {  
    allNodes.clear();   
    //!! Quadtree values must be changed form 0,0 to world's min x and y values else negative coords 
    //!! will not be used in path planning
    //## Divides map into quads to be used for path planning
    doQuadTree(0,0, maxTilesX, maxTilesY, QuadTreeLevel); 
    //## Adds a START and GOAL node to the list of nodes used for path finding
    allNodes.add( new Node(myRobot.location.x, myRobot.location.y, "START", allNodes.size())); 
    allNodes.add( new Node(goalXY.x, goalXY.y, "GOAL", allNodes.size()));
    
    //oldMillis = millis();
    //!! Code must change to only do nodelink when new obstacles are detected and a new quad tree is created
    nodeLink();  //Links all the nodes together in order to determine shortest path
    //time = millis() - oldMillis;
    //println("Node Link time: "+time);
    mapChange = true;
  }
  
  //## Calculate shortest path using A* and the links created with nodeLink
  findPath();
  
  //### Calculates the attractive field for each tile  
  for (int k = 0; k < maxTilesX; k++)
  {
    for (int l = 0; l < maxTilesY; l++)
    {
      if (tile[k][l].tileType == "UNASSIGNED")
      {
        tile[k][l].field.x = calcAttractField(tile[k][l].tilePos.x, tile[k][l].tilePos.y).x + calcRepulsiveField(tile[k][l].tilePos.x, tile[k][l].tilePos.y).x;
        tile[k][l].field.y = calcAttractField(tile[k][l].tilePos.x, tile[k][l].tilePos.y).y + calcRepulsiveField(tile[k][l].tilePos.y, tile[k][l].tilePos.y).y;
      }
    }
  }
  
  //##PlotRobot is the main FSM for the robot. Its used to make decision on what to do next based on the robot position
  //##  and current state of sensors
  PlotRobot();
  
  //## calcProgressPoint tracks the progress point in order to determine if wall following is over
  //calcProgressPoint();
  
  //### Draws cartesian axis on the screen  
  strokeWeight(2);
  stroke(0,255,0);
  line (toScreenX(-1000),toScreenY(0),toScreenX(1000),toScreenY(0));
  line (toScreenX(0), toScreenY(-worldHeight), toScreenX(0), toScreenY(worldHeight));
  
  //## Displays mouse X and Y values in World Coords
  fill(0);  
  textSize(10);
  textAlign(CENTER,BOTTOM);  
  text(toWorldX(mouseX)+":"+toWorldY(mouseY), mouseX, mouseY);
  
  //## Calculates the movement vector based on the robot position and repulsive and attractive forces
  vectorAOFWD.x = (calcAttractField(myRobot.location.x, myRobot.location.y).x + calcRepulsiveField(myRobot.location.x, myRobot.location.y).x);
  vectorAOFWD.y = (calcAttractField(myRobot.location.x, myRobot.location.y).y + calcRepulsiveField(myRobot.location.x, myRobot.location.y).y);
    
  //###Calcualtes the angle in which the robot needs to travel   
  float angleToGoal = atan2(vectorAOFWD.y,vectorAOFWD.x) - myRobot.heading;        
  if (angleToGoal < (-PI)) angleToGoal += 2*PI;
  if (angleToGoal > (PI)) angleToGoal -= 2*PI;
     
  //###Caclualtes the magnitude of the AOFWD vector to determine speed
  float velocityToGoal = 0.0;  
  velocityToGoal = vectorAOFWD.mag();
  
  //??Displays different vectors, ie: Go-To-Goal, Avoid Obstacle, etc
  dispVectors();  
    
  if (simMode)
  {
    //## Shows the framerate in milli seconds on the top of the screen
    oldMillis = newMillis;
    newMillis = millis();
    textSize(16);  
    textAlign(LEFT, TOP);
    fill(0);
    text("frame rate (ms): "+(newMillis - oldMillis),5,5);
    
    //int startTime = millis();
    //##Makes use of sensor class to detect obstacles using the obstacle blocks on the map to determine a simulated
    //##  distance value
    myRobot.sense();   
    //int endTime = millis();
    //println("Sense Time: " + (endTime - startTime));
  }
  else if (!simMode)
  {
    //###Get serial data from robot driver layer: x,y,heading and ultrasonic sensor values
    //inData = "d0:60,1:60,2:60,3:60,4:60,5:60,6:60";
    //inData = "d0:60";
    //parseSerialData();
    
    fill(0,255,0);
    ellipse(toScreenX(int(agent.x)), toScreenY(int(agent.y)), 40 * scaleFactor, 40 * scaleFactor); //<>//
    agent.x += 0.5 * (calcAttractField(agent.x, agent.y).x + calcRepulsiveField(agent.x, agent.y).x);
    agent.y += 0.5 * (calcAttractField(agent.x, agent.y).y + calcRepulsiveField(agent.x, agent.y).y);
    
    //## Show NO TX across robot to indicate no serial data is being transmitted to driver layer    
    if (!allowTX)
    {
      fill(255,0,0);
      textSize(40);
      textAlign(CENTER, BOTTOM);
      text("NO TX", toScreenX(int(myRobot.location.x)), toScreenY(int(myRobot.location.y)));
    }
    
    if (!allowV)
    {
      fill(255,0,0);
      textSize(40);
      textAlign(CENTER, TOP);
      text("NO V", toScreenX(int(myRobot.location.x)), toScreenY(int(myRobot.location.y)));
    }    
    
    //###Routine sends new instructions to driverlayer every delta_t millis
    time = millis();  
    int interval = time - old_time;
    if (interval > delta_t)
    {
      //println("vectorGTG: "+vectorGoToGoal+", vectorAvoidObstacles: "+vectorAvoidObstacles+", vectorAOFWD: "+vectorAOFWD);
      println("velocity: "+velocityToGoal+ ", angle: " + angleToGoal);
      if (allowTX) 
      {
        updateRobot(velocityToGoal, angleToGoal);
        moveAngle = angleToGoal;
        moveSpeed = velocityToGoal;
        updateParticles();
      }
      old_time = time;
    }    
  }  

  //##Display all the particles  
  updateParticles();
  
  //##Calculates the probability value between the robot's sensors and each particle in order to determine where the robot is
  updateParticleProb();
  
  //##Display particles after new probability updates have been done
  displayParticles();
  
  //##Resample particles to get rid of dead particles
  resample();    
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
      //if (tile[x][y].tileType == "MAP" || tile[x][y].tileType == "USER" || tile[x][y].tileType == "KINECT")
      {
        tile[x][y].drawTileForce();
      }
      //textAlign(CENTER, TOP);
      //textSize(10);
      //text(x+":"+y, toScreenX(int(tile[x][y].tilePos.x)), toScreenY(int(tile[x][y].tilePos.y)));
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

void isInFOW() //<>//
{
  float alpha = 0.0;
  float beta = 0.0;
  float gamma =0.0; //<>//
  PVector newKinectPos = transRot(myRobot.location.x, myRobot.location.y, myRobot.heading, kinectPos.x, kinectPos.y); //<>//
  PVector newLeftPoint = transRot(myRobot.location.x, myRobot.location.y, myRobot.heading, leftPoint.x, leftPoint.y);
  PVector newRightPoint = transRot(myRobot.location.x, myRobot.location.y, myRobot.heading, rightPoint.x, rightPoint.y);
  
  line (toScreenX(int(newKinectPos.x)), toScreenY(int(newKinectPos.y)), toScreenX(int(newLeftPoint.x)), toScreenY(int(newLeftPoint.y))); //<>//
  line (toScreenX(int(newKinectPos.x)), toScreenY(int(newKinectPos.y)), toScreenX(int(newRightPoint.x)), toScreenY(int(newRightPoint.y)));
  
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
//Updates each particle according to robot movement
void updateParticles()
{
  //Update particle movement
  for (int i = 0; i < maxParticles; i++)
  {
    particles[i].move(moveAngle, moveSpeed);  
  }

  //displayParticles();
}

//###############################################################################################
//Display particles on the screen
void displayParticles()
{
//Display updated particles
  for (int i=0; i < maxParticles; i++)
  {
    particles[i].display();
  }
}


//###############################################################################################
//Calculates the particles probabilty value based on the virtual sensor data from the particles position in the map
//    towards the obstacles in the map
void updateParticleProb()
{
  for (int k = 0; k < maxParticles; k++)
    {
      particles[k].sense();
      particles[k].measureProb();
    }
}

//###############################################################################################
//Main FSM for robot movement and decisions
void PlotRobot()
{  
  float distanceToTarget = PVector.dist(goalXY, myRobot.location);

  float phi_GTG = 0.0; //calcGoalAngle(vectorGoToGoal.x, vectorGoToGoal.y);
  float phi_AO = calcGoalAngle(vectorAOFWD.x, vectorAOFWD.y);
  //float phi_AO_GTG = calcGoalAngle(vectorAO_GTG[0], vectorAO_GTG[1]);
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
        
        
        //###Draws an ellipse over the next waypoint that must be reached
        stroke(0,0,255);
        strokeWeight(0);
        fill(0,0,255);
        ellipse (toScreenX(int(nextWaypoint.x)), toScreenY(int(nextWaypoint.y)), 20,20);      
        
        //##!!! Hierdie kode is nie reg nie, die go-to-goal angle moet die pushing force van die tile ook in ag neem
        phi_GTG = calcGoalAngle(nextWaypoint.x - myRobot.location.x, nextWaypoint.y - myRobot.location.y);
      }
      calcErrorAngle(phi_GTG);
    }

    if ((!myRobot.makingProgress) && (myRobot.collisionFlag))
    {
      stateVal = 1;
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

  //## when in simulation mode, the robot is updated by simulated data
  if(simMode)
  {
    moveAngle = constrain ((turnGain * errorAngle), -myRobot.maxTurnRate, myRobot.maxTurnRate);    
    
    //println(myRobot.maxTurnRate +" "+(turnGain * errorAngle)+" "+moveAngle + " --> ");
    moveSpeed = min (myRobot.maxSpeed, (moveGain * (distanceToTarget)));
    
    accuDist += moveSpeed*timeScale;
    if (moveSpeed !=0 ) accuTime+=1*timeScale;
    //println("moveSpeed : " + moveSpeed + "\taccuDist : "+accuDist + "\taccuTime : "+accuTime);
    
    myRobot.move(moveAngle, moveSpeed);
  }
  myRobot.display();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//  Calculates a progress point towards the goal.
//  If the robot's distance-to-goal is less than the progress point distance-to-goal,
//      the robot progresses, else do Wall Following

void calcProgressPoint()
{  
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
      if (tile[x][y].tileType == "MAP" || tile[x][y].tileType == "USER" || tile[x][y].tileType == "KINECT")
      {
        if (tile[x][y]. gravity != 0)
        {
          vectorAO.add(tile[x][y].field);
        }
      }
    }
  }
  vectorAO.normalize();  
  
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

///////////////////////////////////////////////////////////////////////////////////////////////
PVector calculateVectorBlendedAOGTG()
{
  PVector result = new PVector();
  float dist = vectorAvoidObstacles.mag();
  float beta = 0.01;          //The smaller this value gets the smaller sigma becomes
  float sigma = 1 - exp(-beta*dist);
  PVector gtgBlend = new PVector();
  PVector aoBlend = new PVector();  
  
  PVector.mult(vectorGoToGoal,sigma, gtgBlend);
  PVector.mult(vectorAvoidObstacles, (1-sigma), aoBlend); 

  result = PVector.add(gtgBlend, aoBlend);
  return result;
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
  ellipse (toScreenX(int(goalXY.x)), toScreenY(int(goalXY.y)), safeZone*3 * scaleFactor, safeZone*3 * scaleFactor);
  stroke(0);
  fill(255);
  ellipse (toScreenX(int(goalXY.x)), toScreenY(int(goalXY.y)), safeZone*2 * scaleFactor, safeZone*2 * scaleFactor);
  stroke(0);
  fill(0);
  ellipse (toScreenX(int(goalXY.x)), toScreenY(int(goalXY.y)), safeZone * scaleFactor, safeZone * scaleFactor);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void dispVectors()
{  
  //Draws a vector pointing away from all the obstacles
  strokeWeight(4);
  stroke(255,0,0);
  line(toScreenX(int(myRobot.location.x)), toScreenY(int(myRobot.location.y)), 
       toScreenX(int(myRobot.location.x + vectorAvoidObstacles.x)), toScreenY(int(myRobot.location.y + vectorAvoidObstacles.y)));
  
  //###Draws a vector straight towards the goal
  strokeWeight(5);
  stroke(255,255, 0);  
  line(toScreenX(int(myRobot.location.x)), toScreenY(int(myRobot.location.y)), 
       toScreenX(int(myRobot.location.x + vectorGoToGoal.x)), toScreenY(int(myRobot.location.y + vectorGoToGoal.y)));
       
  //line(toScreenX(int(myRobot.location.x)), toScreenY(int(myRobot.location.y)), 
  //     toScreenX(int(myRobot.location.x + 100)), toScreenY(int(myRobot.location.y + 100)));
  
  //###Draws a vector which is a blend between the goal and avoid obstacles
  strokeWeight(5);
  stroke(0,0, 255);
  //line(myRobot.location.x, myRobot.location.y, myRobot.location.x + vectorBlendedAOGTG.x * 100, myRobot.location.y + vectorBlendedAOGTG.y * 100);
  line(toScreenX(int(myRobot.location.x)), toScreenY(int(myRobot.location.y)), 
       toScreenX(int(myRobot.location.x + vectorAOFWD.x)), toScreenY(int(myRobot.location.y + vectorAOFWD.y)));
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//This functin displays data on the graphic screen, anders vergeet ek watter knoppies om te gebruik om dinge te laat gebeur

void displayText()
{
  String[] textLines = {"Keys Used:","Step: <n>"};
  
  textSize(ts);
  textAlign(LEFT);
  for (int i = 0; i < textLines.length; i++) 
  {
    text(textLines[i],10,i*ts+ts);
  }
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
//### LEFT mouse button event changes the position of the goal
//### RIGHT mouse button event changes the position of the robot
void mousePressed()
{
  if (mousePressed && (mouseButton == LEFT)) changeGoal();
  
  if (mousePressed && (mouseButton == RIGHT))
  { 
    robotPosOffset.x = toWorldX(int(mouseX));
    robotPosOffset.y = toWorldY(int(mouseY));
    
    //myRobot.location.x = toWorldX(int(mouseX));
    //myRobot.location.y = toWorldY(int(mouseY));

    //###Resets progress point when target is moved to the current mouse position    
    myRobot.progressPoint.x = toWorldX(int(mouseX));
    myRobot.progressPoint.y = toWorldY(int(mouseY));
    myRobot.makingProgress = true;
    accuDist = 0;
    accuTime = 0;
  }
}

//### Zoom world in and out using mousewheel event
void mouseWheel(MouseEvent event)
{
  float e = event.getCount();
  float zoomScale = viewPortWidth * 0.2;
   
  viewPortWidth += zoomScale * e;
  viewPortHeight += zoomScale * e;
  
  vpX -= zoomScale / 2 * e;
  vpY += zoomScale / 2 * e;
  
  scaleFactor = graphicBoxWidth / viewPortWidth;
  
  //### Reloads original image in order to maintain image sharpness after zoom
  //img = loadImage(mapName);         //Loads image 
  //### Resize image to image width and height represented by the world
  img.resize(int(imgWidth), int(imgHeight));
  
  //### Calculate new resolution for img resize values
  float newWidth = imgWidth / viewPortWidth * graphicBoxWidth;
  float newHeight = imgHeight / viewPortHeight * graphicBoxWidth;
  img.resize(int(newWidth), int(newHeight));
  
  //println("vpX: "+vpX+", vpY: "+vpY+", viewPortWidth: "+viewPortWidth+", viewPortHeight: "+viewPortHeight);
  //println(scaleFactor);
}

//Change the goal location everytime the mouse is clicked
void changeGoal()
{  
  goalXY.x = toWorldX(int(mouseX));
  goalXY.y = toWorldY(int(mouseY));

  startX = myRobot.location.x;
  startY = myRobot.location.y;

  stateVal = 1;
  
  //###Resets progress point when target is moved to the current robot position  
  myRobot.progressPoint = myRobot.location;
  myRobot.makingProgress  = true;
  
  //###Changes the GOAL node to new goal position
  for (int k = 0; k < allNodes.size(); k++)
  {
    if (allNodes.get(k).nodeType == "GOAL")
    {
      allNodes.get(k).nodeXPos = goalXY.x;
      allNodes.get(k).nodeYPos = goalXY.y;        
    }
  }
  //###Link all the nodes together again
  nodeLink();
  //###Calculate new shortest route to GOAL
  findPath();
}

void keyPressed()
{
  //### Add support to disable/enable all serial TX comms
  if ((key == 'x') || (key == 'X'))
  {
    allowTX = !allowTX;    
    myPort.write("<w0\r");
    delay(1);
    myPort.write("<v\r");
  }
  
  //###Controls whether the robot can move forward
  if (key == 'v') 
  {
    allowV = !allowV;
    myPort.write("<v0\r");
  }
  
  //###Enables tilt controll for kinect sensor
  if (key == CODED)
  {    
    if (keyCode == UP)
    {
      kinectTilt ++;
      println("Kinect Tilt: "+kinectTilt);
    } else if (keyCode == DOWN)
    {
      kinectTilt --;
    }
    kinectTilt = constrain(kinectTilt, -30, 30);
    kinect.setTilt(kinectTilt);
  }
  
  if (key == 'w')
  {
    vpY += 10;
    if (vpY >= imgHeight + 100) vpY = imgHeight + 100;
    //println(vpY);
  }
    
  if (key == 's')
  {
    vpY -= 10;
    if (vpY <= viewPortHeight - 100) vpY = viewPortHeight - 100;
    //println(vpY);
  }
  
  if (key == 'a')
  {
    vpX -= 10;
    if (vpX <= -100) vpX = -100;
    //println(vpX);
  }
    
  if (key == 'd')
  {
    vpX += 10;
    if (vpX >= (imgWidth + 100 - viewPortWidth)) vpX = (imgWidth + 100 - viewPortWidth);
    //println(vpX);
  }
    
  if (key == ' ') showVal = true;  
  
  //## This key will cycle through one complete cycle of the robot code
  if (key == 'n') 
  {
    println("STEP!");
    nextStep = true;
  }

  //Use this key to enable or disable obstacle
  if (key == 'o')
  {
    int worldMouseX = int(toWorldX(mouseX)/tileSize);
    int worldMouseY = int(toWorldY(mouseY)/tileSize);
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
  
  if (key == ' ')
  {
    agent.x = toWorldX(mouseX);
    agent.y = toWorldY(mouseY);
  }
}



//Implementation of the following website
//http://www.libertybasicuniversity.com/lbnews/nl112/mapcoor.htm

//Creates a viewport which is used to view only a specific area of the world map
//It also plots world coordinates onto the screen i.e: Inverting the y-axis
int toScreenX(float _x)
{
  return int((graphicBoxWidth / viewPortWidth) * (_x - vpX));
}

int toScreenY(float _y)
{
  return int((graphicBoxHeight / viewPortHeight) * (vpY - _y));
}


float toWorldX (int _x)
{
  return ((float(_x) / graphicBoxWidth * viewPortWidth + vpX));  
}

float toWorldY (int _y)
{
  return (vpY - float(_y) / graphicBoxHeight * viewPortHeight) - 1; //removed -1
}


//### Calculates the force flow vector based on the position for a specific x and y coords
//### Implementation of the concepts in the following website and paper:
//  http://www.cs.mcgill.ca/~hsafad/robotics/
//  Michael A. Goodrich, Potential Fields Tutorial, http://borg.cc.gatech.edu/ipr/files/goodrich_potential_fields.pdf
PVector calcAttractField(float _x, float _y)
{
  PVector attractField = new PVector();
  //## Calculates attractive field towards the next waypoint in the path to the goal
  float d = sqrt(pow(nextWaypoint.x - _x, 2) + pow(_y - nextWaypoint.y, 2));
  float angle = atan2(nextWaypoint.y - _y, nextWaypoint.x - _x);
  
  if (d < tileSize)
  {
    attractField.x = 0;
    attractField.y = 0;
  }
  
  if ((d >= tileSize) && (d <= s+tileSize))
  {
    attractField.x = alpha * (d - tileSize) * cos(angle);
    attractField.y = alpha * (d - tileSize) * sin(angle);
  }
  
  if (d > (s+tileSize))
  {
    attractField.x = alpha * s * cos(angle);
    attractField.y = alpha * s * sin(angle);
  }
  return attractField;
}
  
PVector calcRepulsiveField(float _x, float _y)
{
  PVector repulsiveField = new PVector();
  
  for (int k = 0; k < maxTilesX; k++)
  {
    for (int l = 0; l < maxTilesY; l++)
    {
      //### Test to see if tile is an obstacle tile
      if (tile[k][l].tileType != "UNASSIGNED")
      {
        float d = sqrt(pow(tile[k][l].tilePos.x - _x, 2) + pow(_y - tile[k][l].tilePos.y, 2));
        float angle = atan2(tile[k][l].tilePos.y - _y, tile[k][l].tilePos.x - _x);
        
        if ((d >= tileSize) && (d <= obstacleS + tileSize))
        {
          repulsiveField.x += -beta * (obstacleS + tileSize - d) * cos(angle);
          repulsiveField.y += -beta * (obstacleS + tileSize - d) * sin(angle);
        }
        
        if (d > (obstacleS + tileSize))
        {
          repulsiveField.x += 0;
          repulsiveField.y += 0;
        }        
      }
    }
  }
  return repulsiveField;
}