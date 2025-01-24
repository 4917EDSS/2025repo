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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSub extends SubsystemBase {
  // Create the climb motor
  private final TalonFX m_climbMotor = new TalonFX(Constants.CanIds.kClimbMotor);
  // TODO:  Add limit switches and/or absolute encoder

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

    resetPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Manually set the power of the climb motor(s).
   * 
   * @param power power value -1.0 to 1.0
   */
  public void setPower(double power) {
    m_climbMotor.set(power);
  }

  /**
   * Sets the current angle as the zero angle
   */
  public void resetPosition() {
    m_climbMotor.setPosition(0);
  }

  /**
   * Returns the current angular position of the climb arm
   * 
   * @return position in degrees
   */
  public double getPosition() {
    return m_climbMotor.getPosition().getValueAsDouble();
  }

  /**
   * Returns the current angular velocity of the climb arm
   * 
   * @return velocity in degrees per second
   */
  public double getVelocity() {
    return m_climbMotor.getRotorVelocity().getValueAsDouble();
  }

  /**
   * Returns how much current the motor is currently drawing
   * 
   * @return current in amps or -1.0 if motor can't measure current
   */
  public double getElectricalCurrent() {
    return m_climbMotor.getStatorCurrent().getValueAsDouble();
  }
}
