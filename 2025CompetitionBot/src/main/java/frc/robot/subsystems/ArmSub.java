// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.Arm;

public class ArmSub extends SubsystemBase {
  // Create limit switches
  private final DigitalInput m_armUpperLimit = new DigitalInput(Constants.DioIds.kArmUpperLimit);
  private final DigitalInput m_armLowerLimit = new DigitalInput(Constants.DioIds.kArmLowerLimit);
  // Create the intake motor
  private final SparkMax m_armMotor = new SparkMax(Constants.CanIds.kArmMotor, MotorType.kBrushless);

  private final SparkAbsoluteEncoder m_armEncoder = m_armMotor.getAbsoluteEncoder();

  private final ArmFeedforward m_armFeedforward = new ArmFeedforward(0.001, 0.001, 0.0);
  private final PIDController m_PID_NEOController = new PIDController(0.01, 0.001, 0.001); // really needs some tuning
  private double m_targetAngle = 0;

  // private final SparkAbsoluteEncoder m_armAbsoluteEncoder = m_armMotor.getAbsoluteEncoder();

  /** Creates a new ArmSub. */
  public ArmSub() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(false) // Set to true to invert the forward motor direction
        .smartCurrentLimit(5) // Current limit in amps
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(Constants.ArmSub.kArmEncoderPositionConversionFactor)
            .velocityConversionFactor(Constants.ArmSub.kArmEncoderVelocityConversionFactor); // Set to kCoast to allow the motor to coast when power is 0.0

    // Save the configuration to the motor
    // Only persist parameters when configuring the motor on start up as this operation can be slow
    m_armMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    resetEncoder();
  }

  @Override
  public void periodic() {
    runPIDControl();
    // This method will be called once per scheduler run
  }

  public void moveArm(double power) {
    m_armMotor.set(power);
    System.out.println(power);
  }

  public double getAngle() {
    return m_armMotor.getEncoder().getPosition();
  }

  public void setAngle(double targetAngle) {
    if(targetAngle > Constants.Arm.kMaxArmAngle) {
      targetAngle = Constants.Arm.kMaxArmAngle;
    }
    if(targetAngle < Constants.Arm.kMinArmAngle) {
      targetAngle = Constants.Arm.kMinArmAngle;
    }
    m_targetAngle = targetAngle;
  }

  public void resetEncoder() {
    m_armMotor.getEncoder().setPosition(0);
  }

  public double getDistance() {
    return m_armMotor.getEncoder().getPosition();
  }

  public double getVelocity() {
    return m_armMotor.getEncoder().getVelocity();
  }

  public boolean isAtUpperLimit() {
    return m_armUpperLimit.get(); // If switch is normally closed, return !m_armUpperLimit.get()
    // to return a true when switch is false and false when it's true
  }

  public boolean isAtLowerLimit() {
    return m_armLowerLimit.get(); // If switch is normally closed, return !m_armLowerLimit.get()
    // to return a true when switch is false and false when it's true
  }

  public void runPIDControl() {
    double pidPower = m_PID_NEOController.calculate(getAngle(), m_targetAngle);
    double fedPower = m_armFeedforward.calculate(Math.toRadians(getAngle()), pidPower); // Feed forward expects 0 degrees as horizontal


    double pivotPower = pidPower + fedPower;
    // moveArm(pivotPower);

  }
}
