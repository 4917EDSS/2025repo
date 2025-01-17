// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSub extends SubsystemBase {
  // Create the climb motor
  private final SparkMax m_climbMotor = new SparkMax(Constants.CanIds.kClimbMotor, MotorType.kBrushless);

  /** Creates a new ClimbSub. */
  public ClimbSub() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit(5) // Current limit in amps
        .idleMode(IdleMode.kBrake); // Set to kCoast to allow the motor to coast when power is 0.0

    // Save the configuration to the motor
    // Only persist parameters when configuring the motors on start up as this operation can be slow
    m_climbMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    // If we change the parameters (e.g. brake mode) during robot operation, 
    // we should not save the changes to flash (i.e. want kNoPersistParameters)
    //m_intakeMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, 
    //  SparkBase.PersistMode.kNoPersistParameters);
    resetEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Control the climb
  public void moveClimb(double power) {
    m_climbMotor.set(power);
  }

  public void resetEncoder() {
    m_climbMotor.getEncoder().setPosition(0);
  }

  public double getDistance() {
    return m_climbMotor.getEncoder().getPosition();
  }

  public void resetPosition() {

  }

  public double getVelocity() {
    return m_climbMotor.getEncoder().getVelocity();
  }
}
