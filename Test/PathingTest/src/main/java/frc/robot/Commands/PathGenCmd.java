package frc.robot.Commands;

import java.util.ArrayList;
import java.util.Arrays;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathGenCmd{
  int[] currentPos;
  int[] targetPos;
  double g;

  public double calcF(int[] pos){
    return calcG+calcH(pos);
  }

  public double calcH(int[] pos){
    return Math.sqrt(Math.pow(targetPos[0] - pos[0], 2));
  }

  public int calcG(){
    //idfk
  }

  public void generatePath(int[] startingPos, int[] targetPos, int[][] field) {
    this.targetPos = targetPos;
    ArrayList<int[]> toSearch = new ArrayList<int[]>();
    ArrayList<int[]> processed = new ArrayList<int[]>();
    toSearch.add(startingPos);
    while(!currentPos.equals(targetPos)) {
      currentPos = toSearch.get(0);
        for(int[] coord : toSearch){
          if(calcF(coord)<calcF(currentPos) || calcF(coord) == calcF(currentPos) && calcH(coord) < calcH(currentPos)){
            currentPos = coord.clone();
          }
          processed.add(currentPos);
          toSearch.remove(currentPos);

          if(currentPos.equals(targetPos)
        }
    }
  }
}