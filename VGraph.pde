


void VGraph()
{
  //Start at 1 because lefthand column is a border tile and stop at maxTiles - 1 because right hand column are border tiles
  for (int k = 1; k < maxTilesX - 1; k++) //<>//
  {
    for (int l = 1; l < maxTilesY - 1; l++)
    {
      println(k+":"+l);
      if ((tile[k][l].tileType == "MAP" || tile[k][l].tileType == "USER")) ellipse(tile[k][l].tilePos.x, tile[k][l].tilePos.y, 20,20);
    }
  }
  
}
