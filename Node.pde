color allNodesColor = color(211); //Colour of the connections between the nodes
int allNodesStrokeWeight = 1;    //Stroke weight of the connections between the nodes
color finalPathColor = color(0,255,0);
int finalPathStrokeWeight = 10;

class Node
{ 
  int startIdx;
  int endIdx;
  int nodeID;      //A unique ID for each node
  int parentID = -1;    //
  float H = 0.0;
  float F = 0.0;
  float G = 0.0;
  
  // -----  //Calculate h first - dit is 'n konstante waarde van die node tot by die goal
  //add strarting node to closed list
  //add all currenlty linked nodes to open list if not there already
  //calc g score for all linked nodes in open list, from current node to linked node, use straight line distance for cost
  //    store g value in connected node - nie hierdie node nie.
  //    initial g value is straight line distance from current location
  //Calc f for each node: F = g + H
  //Pick node with lowest F score, this becomes current node
  //check all connected nodes to see if their G score is improved by going through current node first. 
      //If yes, then recalculate that nodes G score
  //Pick node with lowest F scrore and make it the current node
  
  
  float nodeXPos, nodeYPos;        //x and y location of node;
  
  boolean nodeActive = false;    //Active node used for path finding
  String nodeType = "";
  
  ArrayList<Integer> nodeConnectedTo = new ArrayList<Integer>();
  

  
  Node(float _nodeXPos, float _nodeYPos)
  {        
    nodeType = "DEFAULT";    //As opposed to START or GOAL
    nodeActive = false;
    
    nodeXPos = _nodeXPos;
    nodeYPos = _nodeYPos;
  }
  
  Node()
  {    
    nodeType = "DEFAULT";    //As opposed to START or END
    nodeActive = false;
  }
  
  Node(int _nodeID)
  {    
    nodeType = "DEFAULT";    //As opposed to START or END
    nodeActive = false;
    nodeID = _nodeID;
  }
  
  Node(float _nodeXPos, float _nodeYPos, String _nodeType)
  {    
    nodeType = _nodeType;    //As opposed to START or GOAL
    nodeActive = false;
    
    nodeXPos = _nodeXPos;
    nodeYPos = _nodeYPos;
  }
  
  Node(float _nodeXPos, float _nodeYPos, String _nodeType, int _nodeID)
  {    
    nodeType = _nodeType;    //As opposed to START or GOAL
    nodeActive = false;
    nodeID = _nodeID;
    
    nodeXPos = _nodeXPos;
    nodeYPos = _nodeYPos;
  }
  
  Node(float _nodeXPos, float _nodeYPos, int _nodeID)
  {    
    nodeType = "DEFAULT";    //As opposed to START or GOAL
    nodeActive = false;
    nodeID = _nodeID;
    
    nodeXPos = _nodeXPos;
    nodeYPos = _nodeYPos;
  }
  
  
  void display()
  {
    strokeWeight(allNodesStrokeWeight);
    stroke(allNodesColor);
    
    switch (nodeType)
    {
      case "START":
        //strokeWeight(1);
        //fill(0,255,0);
        //stroke(0);
        //ellipse(nodeXPos, nodeYPos, 30,30);
        break;
        
      case "GOAL":
        //strokeWeight(1);
        //fill(255,0,0);
        //stroke(0);
        //ellipse(nodeXPos, nodeYPos, 30,30);
        break;
        
      default:     
          strokeWeight(1);
          stroke(0);
          fill(150);
          ellipse(toScreenX(int(nodeXPos)),toScreenY(int(nodeYPos)), 5 * scaleFactor,5 * scaleFactor);   //Draws an ellipse to indicate node x,y        
    }
    
    
    
    //textSize(15);    
    //fill(0);
    //textAlign(CENTER,CENTER);
    //text(nodeID,nodeXPos,nodeYPos-20);
    //textSize(15);
    //text(int(H),nodeXPos+20,nodeYPos+20);  //Bottom Right - distance to GOAL
    
    //text(int(G),nodeXPos-20, nodeYPos+20);  //Bottom Left - from START with path values added
    
    //text(int(H+G),nodeXPos-20, nodeYPos-40);  //Top Left  -  sum of G and H
  }
    
  
  
}