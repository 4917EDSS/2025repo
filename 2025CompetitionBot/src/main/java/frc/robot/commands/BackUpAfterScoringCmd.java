// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.DrivetrainSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BackUpAfterScoringCmd extends DeferredCommand {
  /** Creates a new BackUpAfterScoringCmd. */
  public BackUpAfterScoringCmd(DrivetrainSub drivetrainSub, PathConstraints constraints) {
    super(() -> AutoBuilder.pathfindToPose(
        new Pose2d(2, 6.5, new Rotation2d(0)), // TODO - fill this is in with a pose calculated by us!
        constraints,
        0.0 // Goal end velocity in meters/sec
        ), Set.of(drivetrainSub));
        
  }
}
