// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSub extends SubsystemBase {
  NetworkTable m_networkTable = NetworkTableInstance.getDefault().getTable("limelight");
  ShuffleboardTab m_ShuffleboardTab = Shuffleboard.getTab("Vision");
  GenericEntry m_shuffleboardID, m_shuffleboardTv, m_shuffleboardTx, m_shuffleboardTy, m_shuffleboardTa,
      m_shuffleboardPipeline, m_shuffleboardPipetype;

  NetworkTableEntry m_tid;
  NetworkTableEntry m_tv;
  NetworkTableEntry m_tx;
  NetworkTableEntry m_ty;
  NetworkTableEntry m_ta;
  NetworkTableEntry m_pipeline;
  NetworkTableEntry m_pipetype;
  NetworkTableEntry m_botposeTarget;


  /** Creates a new VisionSub. */
  public VisionSub() {
    m_networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_tid = m_networkTable.getEntry("tid");
    m_tv = m_networkTable.getEntry("tv");
    m_tx = m_networkTable.getEntry("tx");
    m_ty = m_networkTable.getEntry("ty");
    m_ta = m_networkTable.getEntry("ta");
    m_pipeline = m_networkTable.getEntry("getpipe");
    m_pipetype = m_networkTable.getEntry("getpipetype");
    m_botposeTarget = m_networkTable.getEntry("botpose_targetspace");

    m_shuffleboardID = m_ShuffleboardTab.add("ID", 0).getEntry();
    m_shuffleboardTv = m_ShuffleboardTab.add("Sees tag?", 0).getEntry();
    m_shuffleboardTx = m_ShuffleboardTab.add("tag x", 0).getEntry();
    m_shuffleboardTy = m_ShuffleboardTab.add("tag y", 0).getEntry();
    m_shuffleboardTa = m_ShuffleboardTab.add("Area of tag", 0).getEntry();
    m_shuffleboardPipeline = m_ShuffleboardTab.add("Pipeline", 0).getEntry();
    m_shuffleboardPipetype = m_ShuffleboardTab.add("Pipetype", 0).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    long id = m_tid.getInteger(0);
    boolean tv = m_tv.getBoolean(false);
    double x = m_tx.getDouble(0.0);
    double y = m_ty.getDouble(0.0);
    double a = m_ta.getDouble(0.0);
    long pipeline = m_pipeline.getInteger(0);
    String pipetype = m_pipetype.getString(null);
    double[] botpose = m_botposeTarget.getDoubleArray(new double[8]);

    m_shuffleboardID.setInteger(id);
    m_shuffleboardTv.setBoolean(tv);
    m_shuffleboardTx.setDouble(x);
    m_shuffleboardTy.setDouble(y);
    m_shuffleboardTa.setDouble(a);
    m_shuffleboardPipeline.setInteger(pipeline);
    m_shuffleboardPipetype.setString(pipetype);
  }


}
