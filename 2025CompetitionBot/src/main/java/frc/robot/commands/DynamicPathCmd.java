// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class DynamicPathCmd extends Command {
  Pose2d targetPose;
  PathConstraints constraints;
  double endVelocity;
  double targetX;
  double targetY;
  double targetRot;

  /** Creates a new DynamicPathCmd. */
  public DynamicPathCmd(double targetX, double targetY, double targetRot, double endVelocity) {
    this.endVelocity = endVelocity;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPose = new Pose2d(targetX, targetY, Rotation2d.fromDegrees(targetRot));

    // Create the constraints to use while pathfinding
    constraints = new PathConstraints(
        6.21, 3.0,
        Units.degreesToRadians(360), Units.degreesToRadians(720));

    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        endVelocity // Goal end velocity in meters/sec
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Since AutoBuilder is configured, we can use it to build pathfinding commands

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
