class Robot{  
  float heading = random (0, 2*PI);
  float robotDiameter = 0; //diameter; * scaleFactor;  //diameter of chassis
  float noseLength = diameter/2;
  float maxSpeed = 1.0;
  float maxTurnRate = 1.0;  
  boolean collisionFlag = false;
  String nodeType = "";    //ROBOT or PARTICLE
  float prob = 1.0;
  float noiseForward = 0.0;
  float noiseTurn = 0.0;
  float noiseSense = 1.0;  
  boolean makingProgress = false;       //Add progress point in order to show if robot is making progress towards goal
  PVector progressPoint = new PVector();  //Holds the coords for the latest progress point of the robot
  PVector location = new PVector(random(0, screenSizeX), random(0, screenSizeY), random(0, 2*PI));   //Holds the robot x and y pos
 
  ArrayList<Sensor> sensors = new ArrayList<Sensor>();
  
  //Instantiates robot as either a ROBOT or a PARTICLE - rules applly differently to the two
  //Supplies a diameter of the robot in cm's for visualization  
  Robot (String _nodeType, float _diameter)
  {   
    nodeType = _nodeType;   
    robotDiameter = _diameter;
  }
  
  Robot (String _nodeType)
  {   
    nodeType = _nodeType;
  }
  
  void addSensor(int _sensorXPos, int _sensorYPos, float _sensorHAngle)
  {
    sensors.add(new Sensor(_sensorXPos, _sensorYPos, _sensorHAngle));
  }
  
  void set(float tempX, float tempY, float tempHeading)
  {    
    location.x = tempX;
    location.y = tempY;
    heading = tempHeading;
  }
  
  
  //Sets movement, sensor and turn noise to robot model
  void setNoise(float _noiseForward, float _noiseTurn, float _noiseSense)
  {
    noiseForward = _noiseForward;
    noiseTurn = _noiseTurn;
    noiseSense = _noiseSense;
  }
  
//Draws the robot() and plots the sensors positions
  void display()
  {    
    //Displays the heading of the robot as a line from the centerpoint of the robot into the same direction as the heading of the robot
    //if (collisionFlag)
    //{
    //  stroke (255,0,0);
    //} 
    //else
    //{
    //  stroke (0);
    //}
    
    switch (nodeType)
    {
      case "ROBOT":
        stroke(0);
        strokeWeight(1);
        fill(0,255,0);    
        ellipse(toScreenX(location.x), toScreenY(location.y), robotDiameter * scaleFactor, robotDiameter * scaleFactor);
        
        //###Displays sensor from ArrayList on robot avatar
        for (int k = 0; k < sensors.size(); k++)
        {
          fill(255,0,0);          
          sensors.get(k).display(location.x,location.y,heading);
          sensors.get(k).displaySensorData(location.x,location.y,heading);
        }
        
        //###Displays safeDistance in which a 'collision' occurs
        noFill();
        stroke(255,0,0);
        //###Value *2 to convert from radius to diameter
        ellipse(toScreenX(location.x), toScreenY(location.y), safeDistance*2 * scaleFactor, safeDistance*2 * scaleFactor);            
        break;
      
      case "PARTICLE":
        stroke(255,0,0);
        fill(255,0,0);
        ellipse(toScreenX(location.x), toScreenY(location.y), max(1,prob*10), max(1,prob*10));  //Shows a small red dot where the head of the particle is else proportionate to the probability        
        textAlign(CENTER, CENTER);
        fill(0);   
        
        //###Display simulated sensor data on screen
        for (int k = 0; k < sensors.size(); k++)
        {
          fill(255,0,0);          
          sensors.get(k).display(location.x,location.y,heading);
          sensors.get(k).displaySensorData(location.x,location.y,heading);
        }
        break;
    } 
    stroke(0);    
    float noseX = location.x + noseLength * cos(heading); 
    float noseY = location.y + noseLength * sin(heading);
    strokeWeight(2);
    line (toScreenX(location.x), toScreenY(location.y), toScreenX(noseX), toScreenY(noseY));
  }
  

  //Moves the robot and particles  
  void move(float turnAngle, float _forward)
  { 
    heading += turnAngle + randomGaussian() * noiseTurn;  //Add the turnAngle value to the current heading
    if (heading >= (2*PI)) heading -= (2*PI);
    if (heading <= (-2*PI)) heading += (2*PI);    
    
    float distance = _forward + randomGaussian() * noiseForward;
    float newX = location.x + distance * cos(heading);
    float newY = location.y + distance * sin(heading);
    location.x = newX;
    location.y = newY;
    
    //###Allows PARTICLES to live in a continuous world
    if (nodeType == "PARTICLE")
    {
      if (location.x > screenSizeX) location.x =- screenSizeX;
      if (location.x < 0) location.x += screenSizeX;
      if (location.y > screenSizeY) location.y =- screenSizeY;
      if (location.y < 0) location.y += screenSizeY;
    }
  }
  
  //###Calcualtes distances to obstacles for each sensor in the sensor array
  //###  This function is used by the particles to sense distance to 
  void sense()
  {
    for (int k = 0; k < sensors.size(); k++)
    {
      sensors.get(k).sense(location.x,location.y,heading);
      if (sensors.get(k).sensorObstacleDist <= safeDistance) myRobot.collisionFlag = true;      
    }
  }  
  
  //Calculates the probability of how closely a particle's measurements to an obstacle coresponds with that of the robot.
  //A prob value is calculated for each sensors distance which is multiplied to all other probabilities of the specific particle
  //Uses a gausian with:
  //  mu     - Particle's measured distance to a obstacle
  //  sigma  - Particle's measurement noise
  //  x      - Robot's distance measurement of the same sensor  
  void measureProb()
  {        
    prob = 1.0;        //Set probability to maximum value
    float probActual =1.0;
    
    for (int k = 0; k < sensors.size(); k++)
    { 
      float mu = sensors.get(k).sensorObstacleDist;
      float sigma = sensors.get(k).sensorNoise;
      float x = myRobot.sensors.get(k).sensorObstacleDist;      

      prob *= exp(- (pow(mu - x, 2) / pow(sigma,2)/2.0) / sqrt(2*PI * pow(sigma,2)));      
    }    
    //println(prob);
  }
  
}