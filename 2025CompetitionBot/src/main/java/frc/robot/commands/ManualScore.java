// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.RobotState;
import frc.robot.Constants;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ManualScore extends Command {
  /** Creates a new ManiualScore. */
  private String position = RobotState.LastReefPosition();
  public double[] armElevator = new double[2];

  public ManualScore() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(position == "L2") {
      armElevator[0] = Constants.Arm.kL2PostScoreAngle;
      armElevator[1] = Constants.Elevator.kL2PostScoreHeight;
    }
    if(position == "L3") {
      armElevator[0] = Constants.Arm.kL3PostScoreAngle;
      armElevator[1] = Constants.Elevator.kL3PostScoreHeight;
    }
    if(position == "L4") {
      armElevator[0] = Constants.Arm.kL4PostScoreAngle;
      armElevator[1] = Constants.Elevator.kL4PostScoreHeight;
    }
    if(position == "L2L3Algae") {
      armElevator[0] = Constants.Arm.kL2L3AlgaeRemovalPostAngle;
      armElevator[1] = Constants.Elevator.kL2L3AlgaeRemovalPostHeight;
    }
    if(position == "L2L3Algae") {
      armElevator[0] = Constants.Arm.kL3L4AlgaeRemovalPostAngle;
      armElevator[1] = Constants.Elevator.kL3L4AlgaeRemovalPostHeight;
    }
  }

  public double[] heightAngles() {
    return (armElevator);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
