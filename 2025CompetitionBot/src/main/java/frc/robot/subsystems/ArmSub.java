// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSub extends SubsystemBase {
  private final SparkMax m_armMotor = new SparkMax(Constants.CanIds.kArmMotor, MotorType.kBrushless);
  private final DigitalInput m_armLowerLimit = new DigitalInput(Constants.DioIds.kArmLowerLimit);
  private final DigitalInput m_armUpperLimit = new DigitalInput(Constants.DioIds.kArmUpperLimit);
  //private final SparkAbsoluteEncoder m_armEncoder = m_armMotor.getAbsoluteEncoder();  // TODO: Figure out if we'll have one of these or not

  private final ArmFeedforward m_armFeedforward = new ArmFeedforward(0.001, 0.001, 0.0);
  private final PIDController m_armPid = new PIDController(0.01, 0.001, 0.001); // really needs some tuning

  private double m_targetAngle = 0;
  private boolean m_automationEnabled = true;


  /** Creates a new ArmSub. */
  public ArmSub() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(false) // Set to true to invert the forward motor direction
        .smartCurrentLimit(5) // Current limit in amps
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(Constants.Arm.kEncoderPositionConversionFactor)
            .velocityConversionFactor(Constants.Arm.kEncoderVelocityConversionFactor); // Set to kCoast to allow the motor to coast when power is 0.0

    // Save the configuration to the motor
    // Only persist parameters when configuring the motor on start up as this operation can be slow
    m_armMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    resetPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(m_automationEnabled) {
      runAngleControl(false);
    } else {
      runAngleControl(true);
    }
  }

  /**
   * Manually set the power of the arm motor(s).
   * 
   * @param power power value -1.0 to 1.0
   */
  public void setPower(double power) {
    m_armMotor.set(power);
    System.out.println(power);
  }

  /**
   * Sets the current angle as the zero angle
   */
  public void resetPosition() {
    // TODO:  If we have an absolute encoder, use that instead of the motor's internal encoder
    // Would need to save an offset to permit resetting the position
    m_armMotor.getEncoder().setPosition(0);
  }

  /**
   * Returns the current angular position of the arm
   * 
   * @return position in degrees
   */
  public double getPosition() {
    // TODO:  If we have an absolute encoder, use that instead of the motor's internal encoder
    return m_armMotor.getEncoder().getPosition();
  }

  /**
   * Returns the current angular velocity of the arm
   * 
   * @return velocity in degrees per second
   */
  public double getVelocity() {
    // TODO:  If we have an absolute encoder, use that instead of the motor's internal encoder
    return m_armMotor.getEncoder().getVelocity();
  }

  /**
   * Sets the target angle that the arm should move to
   * 
   * @param targetAngle target angle in degrees
   */
  public void setTargetAngle(double targetAngle) {
    if(targetAngle > Constants.Arm.kMaxArmAngle) {
      targetAngle = Constants.Arm.kMaxArmAngle;
    }
    if(targetAngle < Constants.Arm.kMinArmAngle) {
      targetAngle = Constants.Arm.kMinArmAngle;
    }
    m_targetAngle = targetAngle;
  }

  /**
   * Returns if the arm is at its lower limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  public boolean isAtLowerLimit() {
    return m_armLowerLimit.get(); // If switch is normally closed, return !m_armLowerLimit.get()
    // to return a true when switch is false and false when it's true
  }

  /**
   * Returns if the arm is at its upper limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  public boolean isAtUpperLimit() {
    return m_armUpperLimit.get(); // If switch is normally closed, return !m_armUpperLimit.get()
    // to return a true when switch is false and false when it's true
  }

  /**
   * Returns how much current the motor is currently drawing
   * 
   * @return current in amps or -1.0 if motor can't measure current
   */
  public double getElectricalCurrent() {
    return -1.0;
  }

  /**
   * enables automation
   */
  public void enableAutomation() {
    m_automationEnabled = true;
  }

  /**
   * disables automation
   */
  public void disableAutomation() {
    m_automationEnabled = false;
  }

  /**
   * Calculates and sets the current power to apply to the arm to get to or stay at its target
   */
  private void runAngleControl(boolean justCalculate) {
    // TODO:  Fix this
    double pidPower = m_armPid.calculate(getPosition(), m_targetAngle);
    double fedPower = m_armFeedforward.calculate(Math.toRadians(getPosition()), pidPower); // Feed forward expects 0 degrees as horizontal


    if(!justCalculate) {
      setPower(pidPower + fedPower);
    }
  }
}
