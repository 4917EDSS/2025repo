// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSub extends SubsystemBase {
  private final SparkMax m_intakeDeployMotor = new SparkMax(Constants.CanIds.kIntakeDeployMotor, MotorType.kBrushless);
  private final SparkMax m_intakeRollersMotor =
      new SparkMax(Constants.CanIds.kIntakeRollersMotor, MotorType.kBrushless);
  private final DigitalInput m_deployLowerLimit = new DigitalInput(Constants.DioIds.kIntakeLowerLimit);
  private final DigitalInput m_deployUpperLimit = new DigitalInput(Constants.DioIds.kIntakeUpperLimit);

  /** Creates a new IntakeSub. */
  public IntakeSub() {
    SparkMaxConfig configIntakeDeployMotor = new SparkMaxConfig();
    configIntakeDeployMotor
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit(5) // Current limit in amps
        .idleMode(IdleMode.kBrake) // Set to kCoast to allow the motor to coast when power is 0.0
            .encoder
                .positionConversionFactor(Constants.IntakeSub.kIntakeMotorEncoderPositionConversionFactor)
                .velocityConversionFactor(Constants.IntakeSub.kIntakeMotorEncoderVelocityConversionFactor); // Set to kCoast to allow the motor to coast when power is 0.0

    SparkMaxConfig configIntakeRollersMotor = new SparkMaxConfig();
    configIntakeRollersMotor
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit(5) // Current limit in amps
        .idleMode(IdleMode.kBrake) // Set to kCoast to allow the motor to coast when power is 0.0
            .encoder
                .positionConversionFactor(Constants.IntakeSub.kIntakeRaiseMotorEncoderPositionConversionFactor)
                .velocityConversionFactor(Constants.IntakeSub.kIntakeRaiseMotorEncoderVelocityConversionFactor); // Set to kCoast to allow the motor to coast when power is 0.0

    // Save the configuration to the motor
    // Only persist parameters when configuring the motors on start up as this operation can be slow
    m_intakeDeployMotor.configure(configIntakeDeployMotor, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    m_intakeRollersMotor.configure(configIntakeRollersMotor, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    // If we change the parameters (e.g. brake mode) during robot operation, 
    // we should not save the changes to flash (i.e. want kNoPersistParameters)
    //m_intakeMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, 
    //  SparkBase.PersistMode.kNoPersistParameters);
    resetPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Manually set the power of the intake deploy (in/out) motor(s).
   * 
   * @param power power value -1.0 to 1.0
   */
  public void setDeployPower(double power) {
    m_intakeDeployMotor.set(power);
  }

  /**
   * Manually set the power of the intake rollers motor(s).
   * 
   * @param power power value -1.0 to 1.0
   */
  public void setRollersPower(double power) {
    m_intakeRollersMotor.set(power);
  }

  /**
   * Sets the current angle as the zero angle
   */
  public void resetPosition() {
    m_intakeDeployMotor.getEncoder().setPosition(0);
  }

  /**
   * Returns the current angular position of the intake mechanism
   * 
   * @return position in degrees
   */
  public double getPosition() {
    return m_intakeDeployMotor.getEncoder().getPosition();
  }

  /**
   * Returns the current angular velocity of the intake mechanism
   * 
   * @return velocity in degrees per second
   */
  public double getVelocity() {
    return m_intakeDeployMotor.getEncoder().getVelocity();
  }

  /**
   * Returns if the intake mechanism is at its lower (all the way inside) limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  public boolean isAtUpperLimit() {
    return m_deployUpperLimit.get(); // If switch is normally closed, return !m_armUpperLimit.get()
    // to return a true when switch is false and false when it's true
  }

  /**
   * Returns if the intake mechanism is at its upper (all the way outside) limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  public boolean isAtLowerLimit() {
    return m_deployLowerLimit.get(); // If switch is normally closed, return !m_armLowerLimit.get()
    // to return a true when switch is false and false when it's true
  }

  /**
   * Returns how much current the motor is currently drawing
   * 
   * @return current in amps or -1.0 if motor can't measure current
   */
  public double getElectricalCurrent() {
    // TODO:  Figure out how to handle returning current for other motors
    return m_intakeDeployMotor.getOutputCurrent();
  }
}
