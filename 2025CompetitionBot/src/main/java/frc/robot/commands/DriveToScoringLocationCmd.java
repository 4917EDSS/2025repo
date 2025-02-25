// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;
import java.util.Optional;

     import static java.util.Map.entry;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.subsystems.DrivetrainSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToScoringLocationCmd extends SelectCommand {
  public enum ScoringLocation {
    // These color ambiguous ones are meant to go to the color you want (ie. your alliance's color)
    DRIVER_STATION,
    LEFT_CLOSE,
    RIGHT_CLOSE,
    LEFT_FAR,
    RIGHT_FAR,
    FAR,
    RED_DRIVER_STATION,
    RED_LEFT_CLOSE,
    RED_RIGHT_CLOSE,
    RED_LEFT_FAR,
    RED_RIGHT_FAR,
    RED_FAR,
    BLUE_DRIVER_STATION,
    BLUE_LEFT_CLOSE,
    BLUE_RIGHT_CLOSE,
    BLUE_LEFT_FAR,
    BLUE_RIGHT_FAR,
    BLUE_FAR,
  }

  private static Map<ScoringLocation, Command> sLocationToCommandMap;
  public static void warmUpMap(PathConstraints constraints) {
    if (sLocationToCommandMap == null) {
      sLocationToCommandMap = Map.ofEntries(
        entry(ScoringLocation.RED_DRIVER_STATION,  AutoBuilder.pathfindToPose(new Pose2d(0, 0, new Rotation2d(0)), constraints, 0.0)),
        entry(ScoringLocation.RED_LEFT_CLOSE,  AutoBuilder.pathfindToPose(new Pose2d(0, 0, new Rotation2d(0)), constraints, 0.0)),
        entry(ScoringLocation.RED_RIGHT_CLOSE,  AutoBuilder.pathfindToPose(new Pose2d(0, 0, new Rotation2d(0)), constraints, 0.0)),
        entry(ScoringLocation.RED_LEFT_FAR,  AutoBuilder.pathfindToPose(new Pose2d(0, 0, new Rotation2d(0)), constraints, 0.0)),
        entry(ScoringLocation.RED_RIGHT_FAR,  AutoBuilder.pathfindToPose(new Pose2d(0, 0, new Rotation2d(0)), constraints, 0.0)),
        entry(ScoringLocation.RED_FAR,  AutoBuilder.pathfindToPose(new Pose2d(0, 0, new Rotation2d(0)), constraints, 0.0)),
        entry(ScoringLocation.BLUE_DRIVER_STATION,  AutoBuilder.pathfindToPose(new Pose2d(0, 0, new Rotation2d(0)), constraints, 0.0)),
        entry(ScoringLocation.BLUE_LEFT_CLOSE,  AutoBuilder.pathfindToPose(new Pose2d(0, 0, new Rotation2d(0)), constraints, 0.0)),
        entry(ScoringLocation.BLUE_RIGHT_CLOSE,  AutoBuilder.pathfindToPose(new Pose2d(0, 0, new Rotation2d(0)), constraints, 0.0)),
        entry(ScoringLocation.BLUE_LEFT_FAR,  AutoBuilder.pathfindToPose(new Pose2d(0, 0, new Rotation2d(0)), constraints, 0.0)),
        entry(ScoringLocation.BLUE_RIGHT_FAR,  AutoBuilder.pathfindToPose(new Pose2d(0, 0, new Rotation2d(0)), constraints, 0.0)),
        entry(ScoringLocation.BLUE_FAR,  AutoBuilder.pathfindToPose(new Pose2d(0, 0, new Rotation2d(0)), constraints, 0.0))
        );
    }
    PathfindingCommand.warmupCommand().schedule();
  }

  private static ScoringLocation getClosest(DrivetrainSub drivetrainSub) {
    // TODO - find the closest location, and return it.
    return ScoringLocation.DRIVER_STATION;
  }

  private static ScoringLocation cleanLocation(ScoringLocation targetSpot) {
    Optional<Alliance> ally = DriverStation.getAlliance();
    // First, we convert any non-alliance specific locations to alliance-specific ones.
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        if (targetSpot == ScoringLocation.DRIVER_STATION) {
          return ScoringLocation.RED_DRIVER_STATION;
        } else if (targetSpot == ScoringLocation.LEFT_CLOSE) {
          return ScoringLocation.RED_LEFT_CLOSE;
        } else if (targetSpot == ScoringLocation.RIGHT_CLOSE) {
          return ScoringLocation.RED_RIGHT_CLOSE;
        } else if (targetSpot == ScoringLocation.LEFT_FAR) {
          return ScoringLocation.RED_LEFT_FAR;
        } else if (targetSpot == ScoringLocation.RIGHT_FAR) {
          return ScoringLocation.RED_RIGHT_FAR;
        } else if (targetSpot == ScoringLocation.FAR) {
          return ScoringLocation.RED_FAR;
        }
      } else {
        if (targetSpot == ScoringLocation.DRIVER_STATION) {
          return ScoringLocation.BLUE_DRIVER_STATION;
        } else if (targetSpot == ScoringLocation.LEFT_CLOSE) {
          return ScoringLocation.BLUE_LEFT_CLOSE;
        } else if (targetSpot == ScoringLocation.RIGHT_CLOSE) {
          return ScoringLocation.BLUE_RIGHT_CLOSE;
        } else if (targetSpot == ScoringLocation.LEFT_FAR) {
          return ScoringLocation.BLUE_LEFT_FAR;
        } else if (targetSpot == ScoringLocation.RIGHT_FAR) {
          return ScoringLocation.BLUE_RIGHT_FAR;
        } else if (targetSpot == ScoringLocation.FAR) {
          return ScoringLocation.BLUE_FAR;
        }
      }
    }
    return targetSpot;
  }

  /** Command for the closest one. */
  public DriveToScoringLocationCmd(DrivetrainSub drivetrainSub) {
    this(drivetrainSub, getClosest(drivetrainSub));
  }

  public DriveToScoringLocationCmd(DrivetrainSub drivetrainSub, ScoringLocation targetSpot) {
    super(sLocationToCommandMap, () -> cleanLocation(targetSpot));
    assert sLocationToCommandMap != null;
  }
}
