// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;
import java.util.logging.Logger;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.SubControl;
import frc.robot.utils.SubControl.State;
import frc.robot.utils.TestableSubsystem;

public class ArmSub extends TestableSubsystem {
  private static Logger m_logger = Logger.getLogger(ArmSub.class.getName());

  private final SparkMax m_armMotor = new SparkMax(Constants.CanIds.kArmMotor, MotorType.kBrushless);
  private final SparkAbsoluteEncoder m_absoluteEncoder = m_armMotor.getAbsoluteEncoder();
  private final RelativeEncoder m_relativeEncoder = m_armMotor.getEncoder();
  private final SparkLimitSwitch m_forwardLimitSwitch = m_armMotor.getForwardLimitSwitch();
  private final SparkLimitSwitch m_reverseLimitSwitch = m_armMotor.getReverseLimitSwitch();

  // private double m_kS = 0.0;
  // private double m_kG = 0.01;
  // private double m_kV = 0.05;
  // private double m_kP = 0.022;
  // private double m_kI = 0.0;
  // private double m_kD = 0.0;
  // private final ArmFeedforward m_armFeedforward = new ArmFeedforward(m_kS, m_kG, m_kV);
  // private final PIDController m_armPid = new PIDController(m_kP, m_kI, m_kD);

  private final SparkClosedLoopController m_controller = m_armMotor.getClosedLoopController();

  private double m_kMinOutput = -0.5; // -1.0 to 1.0
  private double m_kMaxOutput = 0.5; // -1.0 to 1.0
  private double m_kMaxVel = 3000; // RPM
  private double m_kMaxAccel = 6000; // RPM/s
  private double m_kP = 2.0;
  private double m_kI = 0.0;
  private double m_kD = 0.0;
  private double m_kAllowedError = 0.1; // at-position tolerance, in revolutions

  private Supplier<Double> elevatorPosition;
  private double m_targetAngle = 0;
  private double m_blockedAngle;
  private boolean m_automationEnabled = false;
  private SubControl m_currentControl = new SubControl(); // Current states of mechanism

  /** Creates a new ArmSub. */
  public ArmSub(CanSub canSub) {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(true) // Set to true to invert the forward motor direction
        .smartCurrentLimit(60) // Current limit in amps // TODO: Determine real current limit
        .idleMode(IdleMode.kBrake)
            // Set PID gains
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(m_kP)
                .i(m_kI)
                .d(m_kD)
                .outputRange(m_kMinOutput, m_kMaxOutput)
                    // Set MAXMotion parameters
                    .maxMotion
                        .maxVelocity(m_kMaxVel)
                        .maxAcceleration(m_kMaxAccel)
                        .allowedClosedLoopError(m_kAllowedError);

    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
    encoderConfig.zeroOffset(Constants.Arm.kAbsoluteEncoderOffset);
    motorConfig.apply(encoderConfig);

    // Save the configuration to the motor
    // Only persist parameters when configuring the motor on start up as this
    // operation can be slow
    m_armMotor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    init();
  }

  /**
   * Put the subsystem in a known state
   */
  public void init() {
    m_logger.info("Initializing ArmSub Subsystem");
    m_relativeEncoder.setPosition(getAngle() / Constants.Arm.kArmConversionFactor); // 360 deg / 100 motor rotations per arm rotation
  }

  /**
   * Give this subsystem access to the elevator's current height
   */
  public void setElevatorPositionSupplier(Supplier<Double> elevatorGetPosition) {
    elevatorPosition = elevatorGetPosition;
  }

  @Override
  public void periodic() {
    updateStateMachine();
    runAngleControl();

    SmartDashboard.putNumber("Arm Raw Enc", getPosition());
    SmartDashboard.putNumber("Arm Angle", getAngle());
    SmartDashboard.putBoolean("Arm Lower Limit", isAtLowerLimit());
    SmartDashboard.putBoolean("Arm Upper Limit", isAtUpperLimit());
    SmartDashboard.putNumber("Arm Power", m_armMotor.get());
    SmartDashboard.putNumber("Arm Rel Enc", m_relativeEncoder.getPosition());
    SmartDashboard.putBoolean("Is at Arm limit", isAtTargetAngle());
    SmartDashboard.putNumber("Target Arm Angle", m_targetAngle);
  }

  /**
   * Manually set the power of the arm motor(s).
   * 
   * @param power power value -1.0 to 1.0
   */
  public void setPower(double power) {
    if(power > 1.0) {
      power = 1.0;
    } else if(power < -1.0) {
      power = -1.0;
    }

    m_armMotor.set(power);
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
   * Returns if the arm is at its lower limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  public boolean isAtLowerLimit() {
    return m_reverseLimitSwitch.isPressed();
  }

  /**
   * Returns if the arm is at its upper limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  public boolean isAtUpperLimit() {
    return m_forwardLimitSwitch.isPressed();
  }

  /**
   * Handle the case where the elevator needs to stop due to a potential collision
   */
  private void updateStateMachine() {

    // Determine what power the mechanism should use based on the current state
    switch(m_currentControl.state) {

      case MOVING:
        SmartDashboard.putBoolean("Arm Is Blocked", false);
        // If the mechanism is moving, check if it has arrived at it's target.
        if(isBlocked()) {
          m_blockedAngle = (getAngle());
          m_currentControl.state = State.INTERRUPTED;
        }
        break;

      case INTERRUPTED:
        SmartDashboard.putBoolean("Arm Is Blocked", true);
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

  /**
   * Run the logic to check if the elevator movement is interrupted by a potential
   * collision
   */
  private boolean isBlocked() {

    double armAngle = getAngle();
    double elevatorHeight = elevatorPosition.get();

    if((elevatorHeight <= Constants.Elevator.kDangerZoneBottom) && (armAngle > Constants.Arm.kDangerZoneBottomVertical)
        && (armAngle < Constants.Arm.kDangerZoneLowerAngle)) {
      return true;
    }
    if((elevatorHeight > Constants.Elevator.kDangerZoneBraceBottom)
        && (elevatorHeight < Constants.Elevator.kDangerZoneBraceTop) && (m_targetAngle < armAngle)
        && (armAngle < Constants.Arm.kDangerZoneBraceAngle)) {
      return true;
    }
    return false;
  }

  /**
   * Calculates and sets the current power to apply to the arm to get to or stay at its target
   */
  private void runAngleControl() {
    double currAngle = getAngle();

    if(!m_automationEnabled) {
      // Manual control
      double currentPower = m_armMotor.get();
      if((currAngle >= Constants.Arm.kSlowDownUpperAngle) && (currentPower > Constants.Arm.kSlowDownSpeed)) {
        currentPower = Constants.Arm.kSlowDownSpeed;
      } else if((currAngle < Constants.Arm.kSlowDownLowerAngle) && (currentPower < -Constants.Arm.kSlowDownSpeed)) {
        currentPower = -Constants.Arm.kSlowDownSpeed;
      }

      setPower(currentPower);
    } else {
      // Automation is enabled
      double activeAngle;

      if(m_currentControl.state == State.INTERRUPTED) {
        activeAngle = m_blockedAngle;
      } else {
        activeAngle = m_targetAngle;
      }

      // Convert from degrees to motor rotation
      double rotationPosition = activeAngle / Constants.Arm.kArmConversionFactor; // 360 deg / 100 motor rotations per arm rotation
      m_controller.setReference(rotationPosition, SparkBase.ControlType.kMAXMotionPositionControl);
    }
  }

  /**
   * Indicates whether or not we are at our target height
   * 
   * @return true when we are within tolerance of our target height
   */
  public boolean isAtTargetAngle() {
    // If we are within tolerance and our velocity is low, we're at our target
    if((Math.abs(m_targetAngle - getAngle()) < Constants.Arm.kAngleTolerance)
        && (Math.abs(getVelocity()) < Constants.Arm.kAtTargetMaxVelocity)) {
      return true;
    } else {
      return false;
    }
  }

  //////////////////// Methods used for automated testing ////////////////////
  /**
   * Makes the motor ready for testing. This includes disabling any automation
   * that uses this
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
   * Sets the motor's power to the specified value. This needs to also disable
   * anything else from
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
   * Returns the motor's current encoder value. Ideally this is the raw value, not
   * the converted
   * value. This should be the INTERNAL encoder to minimize dependencies on other
   * hardware.
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
