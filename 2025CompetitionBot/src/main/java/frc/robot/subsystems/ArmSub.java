// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.TestableSubsystem;

public class ArmSub extends TestableSubsystem {
  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Arm");
  private final GenericEntry m_sbArmPower, m_sbLowerLimit, m_sbUpperLimit, m_sbArmPosition, m_sbArmAngle;
  private final SparkMax m_armMotor = new SparkMax(Constants.CanIds.kArmMotor, MotorType.kBrushless);
  private final DigitalInput m_armLowerLimit = new DigitalInput(Constants.DioIds.kArmLowerLimit);
  private final DigitalInput m_armUpperLimit = new DigitalInput(Constants.DioIds.kArmUpperLimit);
  private final SparkAbsoluteEncoder m_absoluteEncoder = m_armMotor.getAbsoluteEncoder();

  private final ArmFeedforward m_armFeedforward = new ArmFeedforward(0.001, 0.001, 0.0);
  private final PIDController m_armPid = new PIDController(0.01, 0, 0); // really needs some tuning

  private double m_targetAngle = 0;
  private boolean m_automationEnabled = false;


  /** Creates a new ArmSub. */
  public ArmSub() {
    m_sbArmPower = m_shuffleboardTab.add("Arm power", 0).getEntry();
    m_sbArmPosition = m_shuffleboardTab.add("Arm position", getPosition()).getEntry();
    m_sbArmAngle = m_shuffleboardTab.add("Arm angle", getPosition()).getEntry();
    m_sbLowerLimit = m_shuffleboardTab.add("Lower limit", isAtLowerLimit()).getEntry();
    m_sbUpperLimit = m_shuffleboardTab.add("Upper limit", isAtUpperLimit()).getEntry();
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit(15) // Current limit in amps
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(Constants.Arm.kEncoderPositionConversionFactor)
            .velocityConversionFactor(Constants.Arm.kEncoderVelocityConversionFactor); // Set to kCoast to allow the motor to coast when power is 0.0

    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
    encoderConfig.zeroOffset(Constants.Arm.kAbsoluteEncoderOffset);
    motorConfig.apply(encoderConfig);

    // Save the configuration to the motor
    // Only persist parameters when configuring the motor on start up as this operation can be slow
    m_armMotor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleBoard();

    if(m_automationEnabled) {
      runAngleControl(false);
    } else {
      runAngleControl(true);
    }
  }

  public void updateShuffleBoard() {
    if(!RobotContainer.disableShuffleboardPrint) {
      m_sbArmPower.setDouble(m_armMotor.get());
      m_sbArmPosition.setDouble(getPosition());
      m_sbArmAngle.setDouble(getAngle());
      m_sbLowerLimit.setBoolean(isAtLowerLimit());
      m_sbUpperLimit.setBoolean(isAtUpperLimit());
    }

    SmartDashboard.putNumber("Arm enc", getPosition());
    SmartDashboard.putNumber("Arm degrees", getAngle());
  }

  /**
   * Manually set the power of the arm motor(s).
   * 
   * @param power power value -1.0 to 1.0
   */
  public void setPower(double power) {
    // Prevent motor from moving past limit switch
    // if((power < 0.0) && isAtLowerLimit()) {
    //   power = 0.0;
    // } else if((power > 0.0) && isAtUpperLimit()) {
    //   power = 0.0;
    // }

    m_armMotor.set(power);
    // System.out.println(power);
  }

  /**
   * Returns the raw current position of the arm
   * 
   * @return position in rotations
   */
  public double getPosition() {

    return m_absoluteEncoder.getPosition(); // returns rotations

  }


  /**
   * Returns the current angular position of the arm
   * 
   * @return position in degrees
   */
  public double getAngle() {

    return m_absoluteEncoder.getPosition() * 360; // returns angle

  }

  /**
   * Returns the current angular velocity of the arm
   * 
   * @return velocity in degrees per second
   */
  public double getVelocity() {
    return m_absoluteEncoder.getVelocity();
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
    if(!m_automationEnabled) {
      m_targetAngle = getAngle();
      m_automationEnabled = true;
    }
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


    double pidPower = m_armPid.calculate(getAngle(), m_targetAngle);
    double fedPower = m_armFeedforward.calculate(Math.toRadians(getAngle()), pidPower); // Feed forward expects 0 degrees as horizontal

    if(!justCalculate) {
      double tempPower = (pidPower + fedPower);

      double SPEED = 0.15;
      if(Math.abs(tempPower) > SPEED) {
        double sign = (tempPower >= 0.0) ? 1.0 : -1.0;
        tempPower = SPEED * sign;
      }
      setPower(tempPower);
    }


    // if(!justCalculate) {
    //   if(getAngle() < m_targetAngle - 2) {
    //     setPower(0.10);
    //   } else if(getAngle() > m_targetAngle + 2) {
    //     setPower(-0.10);
    //   } else {
    //     setPower(0);
    //   }

  }

  //////////////////// Methods used for automated testing ////////////////////
  /**
   * Makes the motor ready for testing. This includes disabling any automation that uses this
   * motor
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  @Override
  public void testEnableMotorTestMode(int motorId) {
    enableAutomation();
  }

  /**
   * Puts the motor back into normal opeation mode.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  @Override
  public void testDisableMotorTestMode(int motorId) {
    // Re-ensable any mechanism automation
    disableAutomation();
  }

  /**
   * Resets the motor's encoder such that it reads zero
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  @Override
  public void testResetMotorPosition(int motorId) {
    switch(motorId) {
      case 1:
        // Do nothing (absolute encoder)
        break;
      default:
        // Do nothing
        break;
    }
  }

  /**
   * Sets the motor's power to the specified value. This needs to also disable anything else from
   * changing the motor power.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   * @param power Desired power -1.0 to 1.0
   */
  @Override
  public void testSetMotorPower(int motorId, double power) {
    switch(motorId) {
      case 1:
        m_armMotor.set(power);
        break;
      default:
        // Do nothing
        break;
    }
  }

  /**
   * Returns the motor's current encoder value. Ideally this is the raw value, not the converted
   * value. This should be the INTERNAL encoder to minimize dependencies on other hardware.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   * @return Encoder value in raw or converted units
   */
  @Override
  public double testGetMotorPosition(int motorId) {
    double angle = 0.0;

    switch(motorId) {
      case 1:
        angle = m_absoluteEncoder.getPosition() * 360; // gives you the angle
        break;
      default:
        // Return an invalid value
        angle = -99999999.0;
        break;
    }

    return angle;
  }

  /**
   * Returns the motor's current current-draw.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   * @return Electrical current draw in amps, or -1 if feature not supported
   */
  @Override
  public double testGetMotorAmps(int motorId) {
    double current = 0.0;

    switch(motorId) {
      case 1:
        current = m_armMotor.getOutputCurrent();
        break;
      default:
        // Return an invalid value
        current = -1.0;
        break;
    }

    return current;
  }
}
