// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSub extends SubsystemBase {
  // Create the climb motor
  private final TalonFX m_climbMotor = new TalonFX(Constants.CanIds.kClimbMotor);

  /** Creates a new ClimbSub. */
  public ClimbSub() {
    TalonFXConfigurator talonFxConfiguarator = m_climbMotor.getConfigurator();

    // This is how you set a current limit inside the motor (vs on the input power supply)
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 40; // Limit in Amps
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFxConfiguarator.apply(limitConfigs);

    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFxConfiguarator.apply(outputConfigs);

    // To configure a second motor to follow the first motor
    //boolean turnOppositeDirectionFromMaster = true; // False if both motors turn in same direction, true to make them turn in opposite directions
    //m_testMotor2.setControl(new Follower(m_testMotor.getDeviceID(), turnOppositeDirectionFromMaster));

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
    m_climbMotor.setPosition(0);
  }

  public double getDistance() {
    return m_climbMotor.getPosition().getValueAsDouble();
  }

  public void resetPosition() {

  }

  public double getVelocity() {
    return m_climbMotor.getRotorVelocity().getValueAsDouble();
  }

  public double getAmps() {
    return m_climbMotor.getStatorCurrent().getValueAsDouble();
  }
}
