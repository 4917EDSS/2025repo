package frc.robot.Commands;

import java.lang.reflect.Field;
import java.util.ArrayList;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathGenCmd{
  int[] currentPos;
  int[] targetPos;
  double g;
  int[][] field;

  public double calcF(int[] pos){
    return calcG()+calcH(pos);
  }

  public double calcH(int[] pos){
    return Math.sqrt(Math.pow(targetPos[0] - pos[0], 2));
  }

  public double calcG(){
    return 0.0;
    //idfk
  }

  public int[][] getNeighbours(int[] pos){
    ArrayList<int[]> neighbours = new ArrayList<int[]>();
    for(int i=-1; i<2; i++){
      for(int j=-1; j<2; j++){
        if(pos[0]+1>=0&&pos[0]+1<=55&&pos[1]+1>=0&&pos[1]+1<=27&&field[pos[0]][pos[1]]==0){
          int [] temp = {pos[0]+i, pos[1]+j};
          neighbours.add(temp);
        }
      }
    }
    int[][] returnArray = new int[neighbours.size()][2];
    return returnArray;
  }

  public void generatePath(int[] startingPos, int[] targetPos, int[][] field) {
    this.targetPos = targetPos;
    this.field = field;
    ArrayList<int[]> toSearch = new ArrayList<int[]>();
    ArrayList<int[]> processed = new ArrayList<int[]>();
    toSearch.add(startingPos);

    while(!currentPos.equals(targetPos)) {
      currentPos = toSearch.get(0);

        for(int[] coord : toSearch) {
          if(calcF(coord)<calcF(currentPos) || calcF(coord) == calcF(currentPos) && calcH(coord) < calcH(currentPos)){
            currentPos = coord.clone();
          }

          processed.add(currentPos);
          toSearch.remove(currentPos);

          if(currentPos.equals(targetPos)) {
            //tspmo ong
          }

          for(int[] neighbour : getNeighbours(currentPos)) {
            
          }

        }
    }
  }
}