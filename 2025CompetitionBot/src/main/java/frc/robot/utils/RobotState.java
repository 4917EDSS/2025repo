// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class RobotState extends Command {
  private static String robotLeftRight = "left";
  private static String robotLastReefPosition = "L2";

  /** Creates a new RobotState. */
  public RobotState() {
    // robotLeftRight = "left";
    // robotLastReefPosition = "L2";
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static void l2() {
    robotLastReefPosition = "L2";
  }

  public static void l3() {
    robotLastReefPosition = "L3";
  }

  public static void l4() {
    robotLastReefPosition = "L4";
  }

  public static void l2L3Algae() {
    robotLastReefPosition = "L2L3Algae";
  }

  public static void l3L4Algae() {
    robotLastReefPosition = "L3L4Algae";
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

  public static String LastReefPosition() {
    return robotLastReefPosition;
  }
}
