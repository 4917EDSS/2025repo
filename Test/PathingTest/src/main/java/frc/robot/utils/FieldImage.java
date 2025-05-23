// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;
import java.util.ArrayList;

/** Add your docs here. */
public class FieldImage {
    ArrayList<double[]> fieldVertices = new ArrayList<double[]>();
    ArrayList<double[]> fieldSides = new ArrayList<double[]>();

    ArrayList<double[]> robotVertices = new ArrayList<double[]>();
    ArrayList<double[]> robotSides = new ArrayList<double[]>();

    public void addVerticies(double x, double y) {
        double[] coords = {x,y};
        fieldVertices.add(coords);
    }

    public void addSide(double x1, double y1, double x2, double y2){
        double[] coords = {x1,y1,x2,y2};
        fieldSides.add(coords);
    }

    public void generateField() {
        //add all vertices and sides here. Use side numbers for vertices
    }

    public void generateRobot() {
        //add robot vertices and sides
    }

    public ArrayList<double[]> getVerts() {
        ArrayList<double[]> vertices = new ArrayList<double[]>();
        vertices.addAll(robotVertices);
        vertices.addAll(fieldVertices);
        return vertices;
    }

    public ArrayList<double[]> getSides() {
        ArrayList<double[]> sides = new ArrayList<double[]>();
        sides.addAll(robotSides);
        sides.addAll(fieldSides);
        return sides;
    }

    //Need to cerate representation of field with verticies of shapes and edges. Use visibility graph pathing with a* search algorithm
}
