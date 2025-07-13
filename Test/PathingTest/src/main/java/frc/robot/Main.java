// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Commands.PathGenCmd;
import frc.robot.utils.FieldImage;

public final class Main {
  private Main() {}

  public static void main(String... args) {
     PathGenCmd pathGenCmd = new PathGenCmd();
     FieldImage fieldImage = new FieldImage();
     int[] s  = {0,0};
     int[] t = {20,20};
     System.out.println("i hate ts");
     for(int[] k : pathGenCmd.generatePath(s, t, fieldImage.field)){
      System.out.println('m');
      System.out.println(k);
     }
     //RobotBase.startRobot(Robot::new);

  }
}
