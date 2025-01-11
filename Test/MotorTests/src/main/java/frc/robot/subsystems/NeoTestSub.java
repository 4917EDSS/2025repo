// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NeoTestSub extends SubsystemBase {
  private final SparkMax m_neoVortexMotor = new SparkMax(Constants.CanIds.kNeoVortexId, MotorType.kBrushless);
  //private final SparkFlex m_neoVortexMotor = new SparkFlex(Constants.CanIds.kNeoVortexId, MotorType.kBrushless);

  /** Creates a new NeoTestSub. */
  public NeoTestSub() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
        .inverted(true)
        .smartCurrentLimit(5)
        .idleMode(IdleMode.kCoast);

    // Only persist parameters when configuring the motor on start up as this operation can be slow
    m_neoVortexMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    // If we change the parameters (e.g. brake mode) during robot operation, 
    // we should not save the changes to flash (i.e. want kNoPersistParameters)
    //m_neoVortexMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public RelativeEncoder getEncoder() {
    return m_neoVortexMotor.getEncoder();
  }

  public void drive(double power) {
    m_neoVortexMotor.set(power);
  }
}
