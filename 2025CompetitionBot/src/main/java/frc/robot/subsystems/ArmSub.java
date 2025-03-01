// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;
import java.util.logging.Logger;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.SubControl;
import frc.robot.utils.SubControl.State;
import frc.robot.utils.TestableSubsystem;

public class ArmSub extends TestableSubsystem {
  private static Logger m_logger = Logger.getLogger(ArmSub.class.getName());
  private final SparkMax m_armMotor = new SparkMax(Constants.CanIds.kArmMotor, MotorType.kBrushless);
  private final SparkAbsoluteEncoder m_absoluteEncoder = m_armMotor.getAbsoluteEncoder();

  private final SparkLimitSwitch m_forwardLimitSwitch = m_armMotor.getForwardLimitSwitch();
  private final SparkLimitSwitch m_revLimitSwitch = m_armMotor.getReverseLimitSwitch();

  private final ArmFeedforward m_armFeedforward = new ArmFeedforward(0.018, 0.01, 0.0);
  private final PIDController m_armPid = new PIDController(0.005, 0, 0); // TODO: Tune

  private double m_targetAngle = 0;
  private boolean m_automationEnabled = false;
  private Supplier<Double> elevatorPosition;

  private SubControl m_currentControl = new SubControl(); // Current states of mechanism
  private double m_blockedAngle;

  /** Creates a new ArmSub. */
  public ArmSub() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit(60) // Current limit in amps // TODO: Determine real current limit
        .idleMode(IdleMode.kBrake).encoder
            .positionConversionFactor(Constants.Arm.kEncoderPositionConversionFactor)
            .velocityConversionFactor(Constants.Arm.kEncoderVelocityConversionFactor);

    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
    encoderConfig.zeroOffset(Constants.Arm.kAbsoluteEncoderOffset);
    motorConfig.apply(encoderConfig);

    // Save the configuration to the motor
    // Only persist parameters when configuring the motor on start up as this operation can be slow
    m_armMotor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  public void setElevatorPositionSupplier(Supplier<Double> elevatorGetPosition) {
    elevatorPosition = elevatorGetPosition;
  }

  public void init() {
    m_logger.info("Initializing ArmSub Subsystem");

  }

  @Override
  public void periodic() {

    updateStateMachine();

    // This method will be called once per scheduler run
    // Smartdashboard widgets
    // Arm power Smartdashboard is in setPower method
    SmartDashboard.putNumber("Arm Raw Enc", getPosition()); // Raw encoder position
    SmartDashboard.putNumber("Arm Ang", getAngle()); // Arm angle

    if(m_automationEnabled) {
      runAngleControl(true);
    } else {
      runAngleControl(false);
    }
  }

  /**
   * Manually set the power of the arm motor(s).
   * 
   * @param power power value -1.0 to 1.0
   */
  public void setPower(double power) {
    m_armMotor.set(power);
    SmartDashboard.putNumber("Arm Power", power); // Arm power
  }

  /**
   * Returns the raw current position of the arm
   * 
   * @return position in rotations
   */
  private double getPosition() {
    return m_absoluteEncoder.getPosition(); // returns rotations
  }


  /**
   * Returns the current angular position of the arm
   * 
   * @return position in degrees
   */
  public double getAngle() {
    // TODO:  Do we need to account for the 0 to 360 rollover?
    return (m_absoluteEncoder.getPosition() * 360) - 303.12; // returns angle
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
    enableAutomation();
    runAngleControl(true);
  }

  /**
   * Returns if the arm is at its lower limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  public boolean isAtLowerLimit() {
    return false; // TODO: Read this from the SparkMax since the switch would be wired directly into it
  }

  /**
   * Returns if the arm is at its upper limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  public boolean isAtUpperLimit() {
    return false; // TODO: Read this from the SparkMax since the switch would be wired directly into it
  }

  /**
   * Enables automation
   */
  public void enableAutomation() {
    m_automationEnabled = true;
  }

  /**
   * Disables automation
   */
  public void disableAutomation() {
    m_automationEnabled = false;
  }

  /**
   * Calculates and sets the current power to apply to the arm to get to or stay at its target
   */
  private void runAngleControl(boolean updatePower) {
    double activeAngle = m_targetAngle;

    if(m_currentControl.state == State.INTERRUPTED) {
      activeAngle = m_blockedAngle;
    }

    double pidPower = m_armPid.calculate(getAngle(), activeAngle);
    double fedPower = m_armFeedforward.calculate(Math.toRadians(getAngle()), pidPower); // Feed forward expects 0 degrees as horizontal

    if(updatePower) {
      double tempPower = (pidPower + fedPower);


      if(Math.abs(tempPower) > Constants.Arm.kMaxPower) {
        double sign = (tempPower >= 0.0) ? 1.0 : -1.0;
        tempPower = Constants.Arm.kMaxPower * sign;
      }
      setPower(tempPower);
    }
  }

  public boolean isAtTargetAngle() {
    // If we are within tolerance and our velocity is low, we're at our target
    // TODO:  Add velocity check
    if((Math.abs(m_targetAngle - getAngle()) < Constants.Arm.kAngleTolerance)
        && (Math.abs(getVelocity()) < Constants.Arm.kAtTargetMaxVelocity)) {
      return true;
    } else {
      return false;
    }
  }

  private boolean isBlocked() {
    double armAngle = getAngle();
    double elevatorHeight = elevatorPosition.get();
    if(elevatorHeight <= Constants.Elevator.kDangerZoneBraceBottom) {
      if(armAngle <= Constants.DangerZones.kArmDangerZoneRange1
          && armAngle >= Constants.DangerZones.kArmDangerZoneRange2) {
        return true;
      }
    } else if(Constants.Elevator.kDangerZoneBraceBottom <= elevatorHeight
        && elevatorHeight <= Constants.Elevator.kDangerZoneBraceTop) { // values in mm, PLEASE CHANGE THEM NOW
      if(armAngle <= Constants.DangerZones.kArmDangerZone1) {
        return true;
      }
    }
    return false;
  }

  private void updateStateMachine() {

    // Determine what power the mechanism should use based on the current state
    switch(m_currentControl.state) {

      case MOVING:
        // If the mechanism is moving, check if it has arrived at it's target.
        if(isBlocked()) {
          m_blockedAngle = (getAngle());
          m_currentControl.state = State.INTERRUPTED;
        }
        break;

      case INTERRUPTED:
        // If the mechanism is no longer blocked, transition to MOVING
        if(!isBlocked()) {
          m_currentControl.state = State.MOVING;
          // Otherwise, hold this position
        }
        break;

      default:
        m_currentControl.state = State.INTERRUPTED;
        break;
    }
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
    disableAutomation();
  }

  /**
   * Puts the motor back into normal opeation mode.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  @Override
  public void testDisableMotorTestMode(int motorId) {
    // Re-ensable any mechanism automation
    enableAutomation();
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
        m_armMotor.getEncoder().setPosition(0.0);
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
    double position = 0.0;

    switch(motorId) {
      case 1:
        position = m_armMotor.getEncoder().getPosition();
        break;
      default:
        // Return an invalid value
        position = -99999999.0;
        break;
    }

    return position;
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
        current = m_armMotor.getOutputCurrent(); // SparkMax doesn't support current reading
        break;
      default:
        // Return an invalid value
        current = -1.0;
        break;
    }

    return current;
  }
}
