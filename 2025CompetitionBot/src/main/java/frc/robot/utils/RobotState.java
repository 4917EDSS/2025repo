// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class RobotState extends Command {
  public enum ReefPosition {
    kL2L3Algae(0), kL3L4Algae(1), kL2(2), kL3(3), kL4(4);

    private int value;

    ReefPosition(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }

  private static String robotLeftRight = "left";
  private static ReefPosition robotLastReefPosition = ReefPosition.kL2;
  public static double[] armElevator = new double[2];


  public static void l2() {
    robotLastReefPosition = ReefPosition.kL2;
    SmartDashboard.putNumber("Last Reef Pos", robotLastReefPosition.getValue());
  }

  public static void l3() {
    robotLastReefPosition = ReefPosition.kL3;
    SmartDashboard.putNumber("Last Reef Pos", robotLastReefPosition.getValue());
  }

  public static void l4() {
    robotLastReefPosition = ReefPosition.kL4;
    SmartDashboard.putNumber("Last Reef Pos", robotLastReefPosition.getValue());
  }

  public static void l2L3Algae() {
    robotLastReefPosition = ReefPosition.kL2L3Algae;
    SmartDashboard.putNumber("Last Reef Pos", robotLastReefPosition.getValue());
  }

  public static void l3L4Algae() {
    robotLastReefPosition = ReefPosition.kL3L4Algae;
    SmartDashboard.putNumber("Last Reef Pos", robotLastReefPosition.getValue());
  }

  public static void setLeft() {
    robotLeftRight = "left";
  }

  public static void setRight() {
    robotLeftRight = "right";
  }

  public static String getSide() {
    return robotLeftRight;
  }

  public static ReefPosition LastReefPosition() {
    return robotLastReefPosition;
  }

  public static double getLastPositionHeight() {
    System.out.println(robotLastReefPosition);
    if(robotLastReefPosition == ReefPosition.kL2) {
      return Constants.Elevator.kL2PostScoreHeight;
    } else if(robotLastReefPosition == ReefPosition.kL3) {
      return Constants.Elevator.kL3PostScoreHeight;
    } else if(robotLastReefPosition == ReefPosition.kL4) {
      return Constants.Elevator.kL4PostScoreHeight;
    } else if(robotLastReefPosition == ReefPosition.kL2L3Algae) {
      return Constants.Elevator.kL2L3AlgaeRemovalPostHeight;
    } else if(robotLastReefPosition == ReefPosition.kL3L4Algae) {
      return Constants.Elevator.kL3L4AlgaeRemovalPostHeight;
    }
    return -1;
  }

  public static double getLastPositionAngle() {
    System.out.println(robotLastReefPosition);
    if(robotLastReefPosition == ReefPosition.kL2) {
      return Constants.Arm.kL2PostScoreAngle;
    } else if(robotLastReefPosition == ReefPosition.kL3) {
      return Constants.Arm.kL3PostScoreAngle;
    } else if(robotLastReefPosition == ReefPosition.kL4) {
      return Constants.Arm.kL4PostScoreAngle;
    } else if(robotLastReefPosition == ReefPosition.kL2L3Algae) {
      return Constants.Arm.kL2L3AlgaeRemovalPostAngle;
    } else if(robotLastReefPosition == ReefPosition.kL3L4Algae) {
      return Constants.Arm.kL3L4AlgaeRemovalPostAngle;
    }
    return 0;
  }
}
