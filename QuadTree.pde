int QuadTreeLevel = 5;
ArrayList<Integer> closedList = new ArrayList<Integer>();
ArrayList<Node> allNodes = new ArrayList<Node>();
ArrayList<Integer> openList = new ArrayList<Integer>();
ArrayList<Integer> finalPath = new ArrayList<Integer>();

//Recursively generates the quad tree nodes
void doQuadTree(int _btmLeftX, int _btmLeftY, int _sizeW, int _sizeH, int _level)
{ 
  //If no mix is found in a quad - draw a node
  if (!findMix(_btmLeftX, _btmLeftY, _sizeW, _sizeH))
  { 
   float nodeX = (_btmLeftX + float(_sizeW)/2)*tileSize;  //cast _sizeW as float in order to do math
   float nodeY = (_btmLeftY + float(_sizeH)/2)*tileSize;  //cast _sizeH as float in order to do math   
      
   allNodes.add(new Node(nodeX,nodeY,allNodes.size()));    //Add new node to allNodes arrayList
   
   //### Draws the nodes and node numbers on the screen
   //stroke(0);
   //fill(0,255,0);
   //ellipse(toScreenX(int(nodeX)),toScreenY(int(nodeY)), 5,5);   //Draws an ellipse to indicate node x,y
   //fill(0);
   //textSize(10);
   //fill(0);
   //text(allNodes.size()-1,toScreenX(int(nodeX-8)),toScreenY(int(nodeY-8)));
   return;
  }
  //If a mixed quad is found and it is the last level DO NOT draw a node, just return
  else if (findMix(_btmLeftX, _btmLeftY, _sizeW, _sizeH) && _level == 0)
  {
    return;
  }
  else
  {
    //If a mixed quad is found: Divide the quad into four new quads   
    //Btm left quad    
    doQuadTree(_btmLeftX, _btmLeftY, _sizeW/2, _sizeH/2, _level-1);
    
    //Btm right quad
    doQuadTree(_btmLeftX + _sizeW/2, _btmLeftY, _sizeW/2, _sizeH/2, _level-1);
    
    //Top left quad    
    doQuadTree(_btmLeftX, _btmLeftY + _sizeH/2, _sizeW/2, _sizeH/2, _level-1);
    
    //Top right quad    
    doQuadTree(_btmLeftX + _sizeW/2, _btmLeftY + _sizeH/2, _sizeW/2, _sizeH/2, _level-1);
  }  
}

//Function returns TRUE if an occupied tile is found within a certain square of tiles
boolean findMix(int _btmLeftX, int _btmLeftY, int _sizeW, int _sizeH)
{
  noFill();
  stroke (0);
  strokeWeight(0);
  
  //###Draws the quad tree divisions on the screen
  rectMode(CORNERS);  
  rect (toScreenX(int(_btmLeftX * tileSize)), toScreenY(int(_btmLeftY * tileSize)), 
        toScreenX(int(_btmLeftX * tileSize + _sizeW * tileSize)), toScreenY(int(_btmLeftY * tileSize + _sizeH * tileSize)));
  
  
  for (int x = 0; x < _sizeW; x++)
  {
    for (int y = 0; y < _sizeH; y++)
    {
      if (tile[x+_btmLeftX][y+_btmLeftY].tileType == "MAP" || tile[x+_btmLeftX][y+_btmLeftY].tileType == "USER")
      {        
        return true;
      }      
    }
  }
  return false;
}

void nodeLink()
{
  boolean intersect = false;
  
  //Looks at each node to find out which other nodes it is connected to
  for (int i = 0; i < allNodes.size(); i++)    //
  {    
    Node n1 = allNodes.get(i);
    
    for (int j = 0; j < allNodes.size(); j++)
    {    
      Node n2 = allNodes.get(j);
        
      for (int k = 0; k < maxTilesX; k++)
      {
        for (int l = 0; l < maxTilesY; l++)
        {
          //Test for intersect between current box and line
          //  If the tile has gravity and the line crosses the tile then the Intersect flag is set
          //if ((tile[k][l].tileType == "MAP" || tile[k][l].tileType == "USER")
          if ((tile[k][l].tileType != "UNASSIGNED")
              && (line_box_xywh(n1.nodeXPos,n1.nodeYPos,n2.nodeXPos,n2.nodeYPos,k*tileSize,l*tileSize,tileSize,tileSize)))
          {
            intersect = true;            
          }
        }
      }
      
      //Test to see if intersect between box and line happend
      //  If it did intersect then reset flag, if not then draw a line to indicate a path between nodes
      if (intersect)
      {        
        intersect = false;
      }
      else
      {        
        //###Draws links to connect nodes together
        //stroke(allNodesColor);   
        //strokeWeight(allNodesStrokeWeight);
        //line(toScreenX(int(n1.nodeXPos)),toScreenY(int(n1.nodeYPos)),toScreenX(int(n2.nodeXPos)),toScreenY(int(n2.nodeYPos)));        
        if(!(n1.nodeXPos == n2.nodeXPos && n1.nodeYPos == n2.nodeYPos))      //Does not link to itself
        {
          n1.nodeConnectedTo.add(j);
        }
       }     
    }
  }
}

void calcH()
{
  for (int k = 0; k < allNodes.size(); k++)
  {
    Node n = allNodes.get(k);
    if (n.nodeType != "GOAL")
    {
      n.H = dist(n.nodeXPos, n.nodeYPos, goalXY.x, goalXY.y);       
    }
  }
}


void findPath()
{ 
  int currentNodeID = 0;
  int startNodeID = 0;
  boolean foundPath = false;
  
  //Clears both lists when starting a new path finding cycle
  openList.clear();
  closedList.clear();
  finalPath.clear();
  
  calcH();      //Function used to calculate the H value of all nodes
  
  //Using the following website: http://homepages.abdn.ac.uk/f.guerin/pages/teaching/CS1013/practicals/aStarTutorial.htm  
  //Step 1
  //Find the START node ID and add it to the openList
  for (int k = 0; k < allNodes.size(); k++)      //Go through the entire allNodes list and find the START node
  {
    
    if (allNodes.get(k).nodeType == "START")
    {
      openList.add(k);                
      currentNodeID = allNodes.get(k).nodeID;  
      startNodeID = currentNodeID;
    }
  }
  
  //println("Current openList: "+openList);
  //println ("Current closedList :"+closedList+"\n");
  
  //LOOP to work through the openList in order to test all possible routes
  while (openList.size() > 0)
  {    
    //Step 2:
    //Adds the nodes connected to current node onto the open list
    //Set nodes in connectedList parentIDs to currentNode
    //Update G values. G = parentNode.G + distance from parent node  
    
    //Find the lowest F score
    float smallestF = 99999.0;           
    
    for (int k=0; k < openList.size(); k++)
    {
      
      if (allNodes.get(openList.get(k)).F < smallestF)
      {
        smallestF = allNodes.get(openList.get(k)).F;
        currentNodeID = openList.get(k);
      }
    }
    
    //println("Closest node is "+currentNodeID+" with F of "+smallestF);
    
    //Move node with lowest F score to closedList
    openList.remove(openList.indexOf(currentNodeID));
    closedList.add(currentNodeID);
    
    Node currentNode = allNodes.get(currentNodeID);            
    for (int k = 0; k < currentNode.nodeConnectedTo.size(); k++)
    { 
      Node openListNode = allNodes.get(currentNode.nodeConnectedTo.get(k));              
      //Test to see if node is in closedList. If it is then it means that, that node
      //    has already been explored and should be ignored
      if (closedList.indexOf(openListNode.nodeID) == -1)
      {
        //Test if nodeConnectedTo number is already in openList, if not then add
        if (openList.indexOf(openListNode.nodeID) == -1)
        {
          openList.add(openListNode.nodeID);      
          openListNode.parentID = currentNodeID;
          openListNode.G = currentNode.G + dist(openListNode.nodeXPos,openListNode.nodeYPos, currentNode.nodeXPos, currentNode.nodeYPos);
          openListNode.F = openListNode.G + openListNode.H;                
          //println("Node "+openListNode.nodeID+"'s H: "+openListNode.H+" G: "+openListNode.G+" and F: "+openListNode.F);                
        }
        else
        {     
          //println(openListNode.nodeID+" already in openList");
          float _dist = dist(currentNode.nodeXPos, currentNode.nodeYPos, openListNode.nodeXPos, openListNode.nodeYPos);
          //println("Distance from "+currentNode.nodeID+" to "+openListNode.nodeID+" is "+_dist);
          //println("Current node G: "+currentNode.G+" openListNode.G: "+openListNode.G);
          //println((currentNode.G + _dist));
          if ((currentNode.G + _dist) < openListNode.G)
          {
            openListNode.G = currentNode.G + _dist;
            openListNode.parentID = currentNode.nodeID;
            //println("Node "+openListNode.nodeID+" new G value = "+openListNode.G);
          }
        }
      }
    }
    
    //println("Current openList: "+openList);
    //println("Current closedList :"+closedList+"\n");
  }
  //LOOP EINDIG HIERSO
  
  //Create final path by starting at the GOAL node and adding parent nodes until the START node is reached
  
  int startPathID = -1;
  
  //Find the ID of the GOAL node
  for (int k = 0; k < allNodes.size(); k++)
  {
    if (allNodes.get(k).nodeType == "GOAL")
    {
      startPathID = allNodes.get(k).nodeID;
      finalPath.add(startPathID);
    }
  }
  
  while( startPathID != startNodeID)
  {
    if (startPathID == -1)
    {
      println ("No Path");
      foundPath = false;
      break;
    }
    else
    {
      startPathID = allNodes.get(startPathID).parentID;
      finalPath.add(startPathID); 
      foundPath = true;
    }
  }
  
  
  //###Draws the best path from robot to goal
  if (foundPath)
  {
    //println("Shortest Path: "+finalPath);
    for (int k = 0; k < finalPath.size()-1; k++)
    {
      strokeWeight(finalPathStrokeWeight);
      stroke(finalPathColor);
      line(toScreenX(int(allNodes.get(finalPath.get(k)).nodeXPos)), toScreenY(int(allNodes.get(finalPath.get(k)).nodeYPos)),
           toScreenX(int(allNodes.get(finalPath.get(k+1)).nodeXPos)), toScreenY(int(allNodes.get(finalPath.get(k+1)).nodeYPos)));
    }
  }
}
