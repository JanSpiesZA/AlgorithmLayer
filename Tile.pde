class Tile
{  
  int gravity = 0;                          
  color gravityCol;
  
  //Tile type variable to distinguish between different types of obstacles in the occupancy grid
  //0 - Unassigned cell
  //1 - Permanent map obstacle
  //2 - user added obstacle
  //3 - kinect obstacle
  String tileType = "UNASSIGNED";    
  
  PVector field;
  PVector tilePos;
  float force = 0.0;
    
  Tile()
  {
    gravityCol = color(150,200,150);
    tileType = "UNASSIGNED";
    field = new PVector();
    tilePos = new PVector();
  }
  
  Tile(float _tileX, float _tileY)
  {
    gravityCol = color(150,200,150);
    tileType = "UNASSIGNED";
    field = new PVector();
    tilePos = new PVector();
    tilePos.x = _tileX;
    tilePos.y = _tileY;
  }
  
  void clearGravity()
  {    
  }
  
  void update()
  {    
    switch(tileType)
    {
      case "UNASSIGNED":
      {
        gravity = 0;
        gravityCol = color(150,200,150);
        //calcAttractField();
        break;
      }
      case "MAP":
      {
        gravity = 255;        
        //calcField();
        gravityCol = color(200,150,150);
        break;
      }
      
      case "USER":
      {
        gravity = 255;        
        //calcField();        
        gravityCol = color(70,130,180);    //steelblue for user obstacles
        break;
      }
      
      case "KINECT":
      {
        gravity = 255;
        //calcField();        
        gravityCol = color(0,191,255);    //deepskyblue for kinect obstacles
        break;
      }
    }
  }
  
  void tileDraw()
  {
    stroke(150);        //Lines between tiles are black
    strokeWeight(1);  //Stroke weight makes the lines very light
    rectMode(CENTER);
    fill(gravityCol,200);
    rect(toScreenX(int(tilePos.x)), toScreenY(int(tilePos.y)), tileSize * scaleFactor, tileSize * scaleFactor);
    
    //###Draws an ellipse according to the spread of the obstacles
    if (tileType != "UNASSIGNED")
    {
      noFill();
      strokeWeight(0);
      stroke(0);
      ellipse(toScreenX(tilePos.x), toScreenY(tilePos.y), obstacleS * scaleFactor, obstacleS * scaleFactor);
    }
    
    
    //####Text to display the real world coords of the tile on the screen
    //textAlign(CENTER,BOTTOM);
    //textSize(10);
    //fill(0);      
    //text(int(tilePos.x)+":"+int(tilePos.y), toScreenX(int(tilePos.x)), toScreenY(int(tilePos.y)));
  }


  void drawTileForce()
  {
    //Draws a flowfield indicator
    strokeWeight(0);
    stroke(0);
    ellipse(toScreenX(int(tilePos.x)), toScreenY(int(tilePos.y)), 1, 1);
    line (toScreenX(int(tilePos.x)), toScreenY(int(tilePos.y)), toScreenX(int(tilePos.x + field.x)), toScreenY(int(tilePos.y + field.y)));    
  }
}