


void VGraph()
{
  //Start at 1 because lefthand column is a border tile and stop at maxTiles - 1 because right hand column are border tiles
  for (int k = 1; k < maxTilesX - 1; k++) //<>//
  {
    for (int l = 1; l < maxTilesY - 1; l++)
    {
      //### Determine if node should be on left top corner
      if ((tile[k][l].tileType == "MAP" || tile[k][l].tileType == "USER"))
        if ((tile[k-1][l].tileType != "USER") && (tile[k-1][l+1].tileType != "USER") && (tile[k][l+1].tileType != "USER"))          
          allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-5),tile[k][l].tilePos.y+tileSize/2+5,allNodes.size()));    //Add new node to allNodes arrayList
          
      //### Determine if node should be on right top corner
      if ((tile[k][l].tileType == "MAP" || tile[k][l].tileType == "USER"))
        if ((tile[k+1][l].tileType != "USER") && (tile[k+1][l+1].tileType != "USER") && (tile[k][l+1].tileType != "USER"))
          allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+5),tile[k][l].tilePos.y+tileSize/2+5,allNodes.size()));    //Add new node to allNodes arrayList
          
      //### Determine if node should be on left bottom corner
      if ((tile[k][l].tileType == "MAP" || tile[k][l].tileType == "USER"))
        if ((tile[k-1][l].tileType != "USER") && (tile[k-1][l-1].tileType != "USER") && (tile[k][l-1].tileType != "USER"))          
          allNodes.add(new Node((tile[k][l].tilePos.x-tileSize/2-5),tile[k][l].tilePos.y-tileSize/2-5,allNodes.size()));    //Add new node to allNodes arrayList
          
       //### Determine if node should be on right bottom corner
      if ((tile[k][l].tileType == "MAP" || tile[k][l].tileType == "USER"))
        if ((tile[k+1][l].tileType != "USER") && (tile[k+1][l-1].tileType != "USER") && (tile[k][l-1].tileType != "USER"))
          allNodes.add(new Node((tile[k][l].tilePos.x+tileSize/2+5),tile[k][l].tilePos.y-tileSize/2-5,allNodes.size()));    //Add new node to allNodes arrayList
    }
  }  
}
