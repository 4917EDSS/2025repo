// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.subsystems.DrivetrainSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToNearestScoreLocationCmd extends SelectCommand<Translation2d> {
  // All in meters.
  // TODO get these exact
  private static final Translation2d MIDDLE_OF_BLUE_REEF = new Translation2d(4917,0);
  private static final Translation2d MIDDLE_OF_RED_REEF = new Translation2d(0,4917);
  private static final double MIDDLE_OF_REEF_TO_SCORING_FACE = 0.4917;
  private static final double MIDDLE_SCORING_FACE_TO_BRANCH = 0.4917;
  private static final double REEF_TO_MIDDLE_OF_ROBOT_SCORING = 0.4917;
  private static List<Pose2d> s_targetPoses = new ArrayList<Pose2d>();
  /**
   * Some basic coordinates summarized from 
   * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
   * 
   * +y
   * ---------------------------
   * | blue              red   |      90
   * |                         |  180    0
   * |                         |     270
   * |0,0                      |     
   * --------------------------- +x
   * 
   * With a top down view where the blue side is the left side, 0,0 is the bottom left corner.
   * x increases going to the right (away from blue towards red)
   * y increases going up (to the left from the perspective of a blue driver)
   * 0 degrees is pointed towards red's driver station
   * 90 degrees is pointed "up"
   * 180 is pointed towards blue
   * 270 is "down"
   */
  private static void generateTargetPoses() {
    for (Translation2d middleOfReef : Arrays.asList(MIDDLE_OF_BLUE_REEF, MIDDLE_OF_RED_REEF)) {
      for (double degrees = 0; degrees < 360; degrees += 60) {
        Rotation2d robotsRotation = Rotation2d.fromDegrees(degrees);
        // The scoring face "looks" at the robot. If the robot is scoring with heading 0 (facing
        // red), the actual scoring side is the 180 angle (towards blue).
        Rotation2d scoringFaceRotation = robotsRotation.minus(Rotation2d.k180deg);
        Translation2d reefToScoringFace = new Translation2d(MIDDLE_OF_REEF_TO_SCORING_FACE + REEF_TO_MIDDLE_OF_ROBOT_SCORING, scoringFaceRotation);
        // TODO - work out what this translation should be. Orthogonal to scoring face.
        Translation2d scoringFaceToBranch = new Translation2d(MIDDLE_SCORING_FACE_TO_BRANCH, 4917);
        s_targetPoses.add(new Pose2d(middleOfReef.plus(reefToScoringFace).plus(scoringFaceToBranch), robotsRotation));
        s_targetPoses.add(new Pose2d(middleOfReef.plus(reefToScoringFace).minus(scoringFaceToBranch), robotsRotation));
      }
    }
    assert s_targetPoses.size() == 2*6*2; // 2 reefs, 6 faces, 2 branches/face
  }

  private static Map<Translation2d, Command> s_locationToCommandMap = new HashMap<Translation2d, Command>();
  public static void warmUpMap(PathConstraints constraints) {
    generateTargetPoses();
    for (Pose2d targetPose : s_targetPoses) {
      s_locationToCommandMap.put(targetPose.getTranslation(), AutoBuilder.pathfindToPose(targetPose, constraints));
    }
    PathfindingCommand.warmupCommand().schedule();
  }

  private static Translation2d getClosest(Pose2d location) {
    return location.getTranslation().nearest(new ArrayList<Translation2d>(s_locationToCommandMap.keySet()));
  }

  /** Command for the closest one. */
  public DriveToNearestScoreLocationCmd(DrivetrainSub drivetrainSub) {
    super(s_locationToCommandMap, () -> getClosest(drivetrainSub.getPose()));
  }
}
