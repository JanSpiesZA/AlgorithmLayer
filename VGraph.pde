
final int nodeDist = 17;

//Implements the concept of Visibility Graphs form Embedded Robotics, Braunl, p254
void VGraph()
{  
  //Start at 1 because lefthand column is a border tile and stop at maxTiles - 1 because right hand column are border tiles  
  for (int k = 1; k < maxTilesX - 1; k++)
  {
    for (int l = 1; l < maxTilesY - 1; l++)
    { 
      int sum = 0;
      if (tile[k][l].tileType != "UNASSIGNED")
      {
        if (tile[k-1][l+1].tileType != "UNASSIGNED") sum += 1;
        if (tile[k][l+1].tileType != "UNASSIGNED") sum += 2;
        if (tile[k+1][l+1].tileType != "UNASSIGNED") sum += 4;
        if (tile[k-1][l].tileType != "UNASSIGNED") sum += 8;
        if (tile[k+1][l].tileType != "UNASSIGNED") sum += 16;
        if (tile[k-1][l-1].tileType != "UNASSIGNED") sum += 32;
        if (tile[k][l-1].tileType != "UNASSIGNED") sum += 64;
        if (tile[k+1][l-1].tileType != "UNASSIGNED") sum += 128;        
        
        switch(sum)
        {
          case 0:
            allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-nodeDist),tile[k][l].tilePos.y+tileSize/2+nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+nodeDist),tile[k][l].tilePos.y+tileSize/2+nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-nodeDist),tile[k][l].tilePos.y-tileSize/2-nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+nodeDist),tile[k][l].tilePos.y-tileSize/2-nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            break;
          case 1:
            allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+nodeDist),tile[k][l].tilePos.y+tileSize/2+nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-nodeDist),tile[k][l].tilePos.y-tileSize/2-nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+nodeDist),tile[k][l].tilePos.y-tileSize/2-nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            break;                   
          case 2: case 3: case 5: case 6: case 7:
            allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-nodeDist),tile[k][l].tilePos.y-tileSize/2-nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+nodeDist),tile[k][l].tilePos.y-tileSize/2-nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            break;          
          case 4:
            allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-nodeDist),tile[k][l].tilePos.y+tileSize/2+nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-nodeDist),tile[k][l].tilePos.y-tileSize/2-nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+nodeDist),tile[k][l].tilePos.y-tileSize/2-nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            break;
          case 8: case 9: case 33: case 40: case 41:
            allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+nodeDist),tile[k][l].tilePos.y+tileSize/2+nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+nodeDist),tile[k][l].tilePos.y-tileSize/2-nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            break;
          case 10: case 11: case 12: case 15: case 34:
            allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+nodeDist),tile[k][l].tilePos.y-tileSize/2-nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            break;
          case 16: case 20: case 132: case 144: case 148:
            allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-nodeDist),tile[k][l].tilePos.y+tileSize/2+nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-nodeDist),tile[k][l].tilePos.y-tileSize/2-nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            break;
          case 17: case 18: case 21: case 22: case 23: case 130: case 134: case 150:
            allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-nodeDist),tile[k][l].tilePos.y-tileSize/2-nodeDist,allNodes.size()));    //Add new node to allNodes arrayList  
            break;            
          case 32:
            allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-nodeDist),tile[k][l].tilePos.y+tileSize/2+nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+nodeDist),tile[k][l].tilePos.y+tileSize/2+nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+nodeDist),tile[k][l].tilePos.y-tileSize/2-nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            break; 
          case 64: case 96: case 160: case 192: case 224:
            allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-nodeDist),tile[k][l].tilePos.y+tileSize/2+nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+nodeDist),tile[k][l].tilePos.y+tileSize/2+nodeDist,allNodes.size()));    //Add new node to allNodes arrayList            
            break;
          case 65: case 72: case 97: case 104: case 105: case 136: case 232:
            allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+nodeDist),tile[k][l].tilePos.y+tileSize/2+nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            break;
          case 48: case 80: case 208: case 212: case 240:
            allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-nodeDist),tile[k][l].tilePos.y+tileSize/2+nodeDist,allNodes.size()));    //Add new node to allNodes arrayList            
            break;
          case 128:
            allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-nodeDist),tile[k][l].tilePos.y+tileSize/2+nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+nodeDist),tile[k][l].tilePos.y+tileSize/2+nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-nodeDist),tile[k][l].tilePos.y-tileSize/2-nodeDist,allNodes.size()));    //Add new node to allNodes arrayList
            break;
          }
        }     
      }
    }
  }
