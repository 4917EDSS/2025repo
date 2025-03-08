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
  String robotLeftRight;

  /** Creates a new RobotState. */
  public RobotState() {
    robotLeftRight = "left";
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void setLeft() {
    robotLeftRight = "left";
  }

  public void setRight() {
    robotLeftRight = "right";
  }

  public String getSide() {
    return robotLeftRight;
  }
}
