// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class IntakeSub extends SubsystemBase {
  // Create limit switches
  private final DigitalInput m_intakeUpperLimit = new DigitalInput(Constants.DioIds.kIntakeLowerLimit);
  private final DigitalInput m_intakeLowerLimit = new DigitalInput(Constants.DioIds.kIntakeLowerLimit);
  // Create the intake motors
  private final SparkMax m_intakeMotor = new SparkMax(Constants.CanIds.kIntakeMotor, MotorType.kBrushless); //motor that intakes coral from field
  private final SparkMax m_intakeRaiseMotor = new SparkMax(Constants.CanIds.kIntakeMotor, MotorType.kBrushless); //motor that intakes coral into robot

  /** Creates a new IntakeSub. */
  public IntakeSub() {
    SparkMaxConfig configIntakeMotor = new SparkMaxConfig();

    configIntakeMotor
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit(5) // Current limit in amps
        .idleMode(IdleMode.kBrake) // Set to kCoast to allow the motor to coast when power is 0.0
            .encoder
                .positionConversionFactor(Constants.IntakeSub.kIntakeMotorEncoderPositionConversionFactor)
                .velocityConversionFactor(Constants.IntakeSub.kIntakeMotorEncoderVelocityConversionFactor); // Set to kCoast to allow the motor to coast when power is 0.0


    SparkMaxConfig configIntakeRaiseMotor = new SparkMaxConfig();

    configIntakeRaiseMotor
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit(5) // Current limit in amps
        .idleMode(IdleMode.kBrake) // Set to kCoast to allow the motor to coast when power is 0.0
            .encoder
                .positionConversionFactor(Constants.IntakeSub.kIntakeRaiseMotorEncoderPositionConversionFactor)
                .velocityConversionFactor(Constants.IntakeSub.kIntakeRaiseMotorEncoderVelocityConversionFactor); // Set to kCoast to allow the motor to coast when power is 0.0

    // Save the configuration to the motor
    // Only persist parameters when configuring the motors on start up as this operation can be slow
    m_intakeMotor.configure(configIntakeMotor, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    m_intakeRaiseMotor.configure(configIntakeRaiseMotor, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

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

  // Control the intake
  public void startIntake(double power) {
    m_intakeMotor.set(power);
  }

  public void resetEncoder() {
    m_intakeMotor.getEncoder().setPosition(0);
  }

  public double getPosition() {
    return m_intakeMotor.getEncoder().getPosition();
  }

  public double getVelocity() {
    return m_intakeMotor.getEncoder().getVelocity();
  }

  public double getAmps() {
    return m_intakeMotor.getOutputCurrent();
  }

  public boolean isAtUpperLimit() {
    return m_intakeUpperLimit.get(); // If switch is normally closed, return !m_armUpperLimit.get()
    // to return a true when switch is false and false when it's true
  }

  public boolean isAtLowerLimit() {
    return m_intakeLowerLimit.get(); // If switch is normally closed, return !m_armLowerLimit.get()
    // to return a true when switch is false and false when it's true
  }
}
