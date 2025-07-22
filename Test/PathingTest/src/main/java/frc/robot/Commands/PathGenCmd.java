package frc.robot.Commands;

import java.util.ArrayList;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class PathGenCmd {
  int[] currentPos;
  int[] targetPos;
  double g;
  int[][] field;
  int[][][] connections;
  double[][] gVals;

  public double calcF(int[] pos) {
    return gVals[pos[0]][pos[1]] + calcH(pos);
  }

  public double calcH(int[] pos) {
    return Math.sqrt(Math.pow(pos[0] - targetPos[0], 2) + Math.pow(pos[1] - targetPos[1], 2));
  }

  public ArrayList<int[]> getNeighbours(int[] pos, ArrayList<int[]> processed) {
    ArrayList<int[]> neighbours = new ArrayList<int[]>();
    for(int i = -1; i < 2; i++) {
      for(int j = -1; j < 2; j++) {
        int[] temp = {pos[0] + i, pos[1] + j};

        if((pos[0] + i >= 0 && pos[0] + i <= 55 && pos[1] + j >= 0 && pos[1] + j <= 27
            && field[pos[0] + i][pos[1] + j] == 0 && (i != 0 || j != 0))) {
          boolean fail = false;
          for(int[] f : processed) {
            if(((temp[0]) == (f[0]) && (temp[1]) == (f[1]))) {
              fail = true;
            }
          }
          if(fail == false) {
            neighbours.add(temp);
          }
        }
      }
    }

    return neighbours;
  }

  public double getDistance(int[] start, int[] end) {
    return Math.sqrt(Math.pow(start[0] - end[0], 2) + Math.pow(start[1] - end[1], 2));
  }

  public ArrayList<int[]> generatePath(int[] startingPos, int[] targetPos, int[][] field) {
    this.targetPos = targetPos;
    this.field = field;
    this.connections = new int[field.length][field[0].length][2];
    gVals = new double[field.length][field[0].length];

    ArrayList<int[]> toSearch = new ArrayList<int[]>();
    ArrayList<int[]> processed = new ArrayList<int[]>();
    toSearch.add(startingPos);

    while(!toSearch.isEmpty()) {
      currentPos = toSearch.get(0);

      for(int[] coord : toSearch) {
        if(calcF(coord) < calcF(currentPos)
            || Math.round((calcF(coord) * 100000)) == Math.round(calcF(currentPos) * 100000)
                && Math.round(calcH(coord) * 100000) < Math.round(calcH(currentPos) * 100000)) {
          currentPos = coord.clone();
        }
      }

      processed.add(currentPos);
      for(int r = 0; r < toSearch.size(); r++) {
        if(toSearch.get(r)[0] == currentPos[0] && toSearch.get(r)[1] == currentPos[1]) {
          toSearch.remove(r);
        }
        if(toSearch.size() > 0) {
        }
      }

      if((currentPos[0]) == (targetPos[0]) && (currentPos[1]) == (targetPos[1])) {
        int[] currentPathCoord = targetPos.clone();
        ArrayList<int[]> path = new ArrayList<int[]>();
        int count = 1000;
        while(!(currentPathCoord[0] == startingPos[0] && currentPathCoord[1] == startingPos[1])) {
          path.add(currentPathCoord);
          int[] tempCoord = currentPathCoord.clone();
          tempCoord[0] = connections[currentPathCoord[0]][currentPathCoord[1]][0];
          tempCoord[1] = connections[currentPathCoord[0]][currentPathCoord[1]][1];
          System.out.println(tempCoord[0] + ", " + tempCoord[1]);
          currentPathCoord = tempCoord.clone();
          count--;
          if(count < 0) {
            //it dies but i dont want to deal with this yet
          }
        }
        path.add(currentPathCoord);
        return path;

      }

      for(int[] neighbour : getNeighbours(currentPos, processed)) {
        boolean inToSearch = toSearch.contains(neighbour);

        double costToNeighbour = gVals[currentPos[0]][currentPos[1]] + getDistance(currentPos, neighbour);

        if(!inToSearch || costToNeighbour < gVals[neighbour[0]][neighbour[1]]) {
          gVals[neighbour[0]][neighbour[1]] = costToNeighbour;
          connections[neighbour[0]][neighbour[1]][0] = currentPos[0];
          connections[neighbour[0]][neighbour[1]][1] = currentPos[1];

          if(!inToSearch) {
            toSearch.add(neighbour);
          }
        }
      }
    }
    ArrayList<int[]> nothing = new ArrayList<int[]>();
    return nothing;
  }


  public String printPoints(int[] startingCoords, int[] targetCoords, int[][] gameField) {
    String coords = "";
    for(int[] n : generatePath(startingCoords, targetCoords, gameField)) {
      coords = coords + (n[0] + ", " + n[1] + "\n");
    }
    return coords;
  }

}
