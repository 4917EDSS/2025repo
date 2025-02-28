// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.logging.Logger;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

public class VisionSub extends SubsystemBase {
  private static String LEFT = "limelight-left";
  private static String RIGHT = "limelight-right";
  private static Logger m_logger = Logger.getLogger(VisionSub.class.getName());

  LimelightHelpers.PoseEstimate mt2;
  Map<String, Double> m_previousTimestamps = Map.of(LEFT, 0.0, RIGHT, 0.0);
  DrivetrainSub m_drivetrainSub;
  NetworkTable m_networkTableL = NetworkTableInstance.getDefault().getTable("limelight-left");
  NetworkTable m_networkTableR = NetworkTableInstance.getDefault().getTable("limelight-right");

  ShuffleboardTab m_ShuffleboardTab = Shuffleboard.getTab("Vision");
  GenericEntry m_shuffleboardID, m_shuffleboardTv, m_shuffleboardT2d, m_shuffleboardTx, m_shuffleboardTy,
      m_shuffleboardTa,
      m_shuffleboardPipeline, m_shuffleboardPipetype;

  NetworkTableEntry m_tid;
  NetworkTableEntry m_t2d;
  NetworkTableEntry m_tv;
  NetworkTableEntry m_tx;
  NetworkTableEntry m_ty;
  NetworkTableEntry m_ta;
  NetworkTableEntry m_pipeline;
  NetworkTableEntry m_pipetype;
  NetworkTableEntry m_botposeTarget;
  NetworkTableEntry m_botpose;

  long id;
  double[] t2d;
  long tv;
  double x;
  double y;
  double a;
  long pipeline;
  String pipetype;
  double[] botposeTarget;
  double[] botpose;

  int m_printPosCounter = 0;


  /** Creates a new VisionSub. */
  public VisionSub(DrivetrainSub drivetrainSub) {
    // For now, we will just use the left camera for shuffleboard.
    // TODO - add the right camera in here.
    m_t2d = m_networkTableL.getEntry("t2d");
    m_tid = m_networkTableL.getEntry("tid");
    m_tv = m_networkTableL.getEntry("tv");
    m_tx = m_networkTableL.getEntry("tx");
    m_ty = m_networkTableL.getEntry("ty");
    m_ta = m_networkTableL.getEntry("ta");
    m_pipeline = m_networkTableL.getEntry("getpipe");
    m_pipetype = m_networkTableL.getEntry("getpipetype");
    m_botposeTarget = m_networkTableL.getEntry("botpose_targetspace");
    m_botpose = m_networkTableL.getEntry("botpose");

    m_shuffleboardID = m_ShuffleboardTab.add("Primary ID", 0).getEntry();
    m_shuffleboardTv = m_ShuffleboardTab.add("Sees tag?", 0).getEntry();
    m_shuffleboardT2d = m_ShuffleboardTab.add("# of Tags", 0).getEntry();
    m_shuffleboardTx = m_ShuffleboardTab.add("tag x", 0).getEntry();
    m_shuffleboardTy = m_ShuffleboardTab.add("tag y", 0).getEntry();
    m_shuffleboardTa = m_ShuffleboardTab.add("Area of tag", 0).getEntry();
    m_shuffleboardPipeline = m_ShuffleboardTab.add("Pipeline", -1).getEntry();
    m_shuffleboardPipetype = m_ShuffleboardTab.add("Pipetype", "unknown").getEntry();

    m_drivetrainSub = drivetrainSub;
    init();
  }

  public void init() {
    m_logger.info("Initializing VisionSub Subsystem");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    id = m_tid.getInteger(0);
    t2d = m_t2d.getDoubleArray(new double[2]);
    tv = m_tv.getInteger(0);
    x = m_tx.getDouble(0.0);
    y = m_ty.getDouble(0.0);
    a = m_ta.getDouble(0.0);
    pipeline = m_pipeline.getInteger(0);
    pipetype = m_pipetype.getString("");
    botposeTarget = m_botposeTarget.getDoubleArray(new double[8]);
    botpose = m_botpose.getDoubleArray(new double[8]);

    m_shuffleboardID.setInteger(id);
    m_shuffleboardT2d.setDouble(t2d[1]);
    m_shuffleboardTv.setInteger(tv);
    m_shuffleboardTx.setDouble(x);
    m_shuffleboardTy.setDouble(y);
    m_shuffleboardTa.setDouble(a);
    m_shuffleboardPipeline.setInteger(pipeline);
    m_shuffleboardPipetype.setString(pipetype);

    updateOdometry(m_drivetrainSub.getState());
  }

  public Pose2d getTagPose2d() {
    return new Pose2d(botposeTarget[0], botposeTarget[2], new Rotation2d(botposeTarget[4]));
  }

  public long getTv() {
    return tv;
  }

  public double getTx() {
    return x;
  }

  private void updateOdometry(SwerveDriveState swerveDriveState) {
    updateOdemetry(swerveDriveState, LEFT);
    updateOdemetry(swerveDriveState, RIGHT);
  }

  public Pose2d getRobotPose() {
    return mt2.pose;
  }

  private void updateOdemetry(SwerveDriveState swerveDriveState, String camera) {
    LimelightHelpers.SetRobotOrientation(camera, swerveDriveState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);
    double timestamp = mt2.timestampSeconds;

    if(timestamp != m_previousTimestamps.get(camera)) {
      boolean doRejectUpdate = false;
      double standardDeviation = 0.7; // 0.7 is a good starting value according to limelight docs.

      if(Math.abs(swerveDriveState.Speeds.omegaRadiansPerSecond) > Math.PI) // if our angular velocity is greater than 360 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
      // TODO:
      // Lower uncertainty (standardDevation) when:
      //   - we see more tags (the more, the better)
      //   - we see a big tag (the bigger the better)
      //   - we are moving slowly (slower is better)
      // Raise uncertainty (standardDeviation) when the opposites happen
      if(!doRejectUpdate) {
        m_drivetrainSub.addVisionMeasurement(
            mt2.pose,
            // Always pass 999999 as the last argument, as megatag 2 requires heading as input, so it does not actually calculate heading.
            // Passing in a very large number to that parameter basically tells the Kalman filter to ignore our calculated heading.
            com.ctre.phoenix6.Utils.fpgaToCurrentTime(timestamp),
            VecBuilder.fill(standardDeviation, standardDeviation, 9999999));
      }
      m_previousTimestamps.replace(camera, timestamp);
    }

  }

}
