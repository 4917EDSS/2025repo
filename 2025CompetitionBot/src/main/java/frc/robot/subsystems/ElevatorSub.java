// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;
import java.util.logging.Logger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.SubControl;
import frc.robot.utils.SubControl.State;
import frc.robot.utils.TestableSubsystem;

public class ElevatorSub extends TestableSubsystem {
  /* STATE VARIABLES */
  private SubControl m_currentControl = new SubControl(); // Current states of mechanism
  private SubControl m_newControl = new SubControl(); // New state to copy to current state when newStateParameters is true
  private boolean m_newControlParameters = false; // Set to true when ready to switch to new state
  private double m_lastPower = -999;
  private double m_blockedPosition;
  private IntakeSub m_intakeSub;
  private LedSub m_ledSub;


  private static Logger m_logger = Logger.getLogger(ElevatorSub.class.getName());
  private final TalonFX m_elevatorMotor = new TalonFX(Constants.CanIds.kElevatorMotor);
  private final TalonFX m_elevatorMotor2 = new TalonFX(Constants.CanIds.kElevatorMotor2);
  private final DigitalInput m_elevatorUpperLimit = new DigitalInput(Constants.DioIds.kElevatorUpperLimit);
  private final DigitalInput m_encoderResetSwitch = new DigitalInput(Constants.DioIds.kElevatorEncoderResetSwitch);

  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.0, 0.06, 0.0);
  private PIDController m_elevatorPID = new PIDController(0.01, 0.0, 0.0);

  private double m_targetHeight = 0.0;
  private boolean m_enableAutomation = false;
  private boolean m_isElevatorEncoderSet = false;
  private int m_hitEncoderSwitchCounter = 0;
  private double m_preTestHeight = 0;
  private double m_preTestHeight2 = 0;
  private Supplier<Double> m_armAngle;


  /** Creates a new ElevatorSub. */
  public ElevatorSub() {

    TalonFXConfigurator talonFxConfiguarator = m_elevatorMotor.getConfigurator();
    TalonFXConfigurator talonFxConfiguarator2 = m_elevatorMotor2.getConfigurator();

    // This is how you set a current limit inside the motor (vs on the input power supply)
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 80; // Limit in Amps  // TODO:  Determine reasonable limit
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFxConfiguarator.apply(limitConfigs);
    talonFxConfiguarator2.apply(limitConfigs);

    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFxConfiguarator.apply(outputConfigs);
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    talonFxConfiguarator2.apply(outputConfigs);

    m_elevatorMotor2.setControl(new Follower(m_elevatorMotor.getDeviceID(), false));

    // Set the encoder conversion factor
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = Constants.Elevator.kRotationsToMm;
    m_elevatorMotor.getConfigurator().apply(config);
    m_elevatorMotor2.getConfigurator().apply(config);

    init();
  }

  public void setArmAngleSupplier(Supplier<Double> armAngle) {
    m_armAngle = armAngle;
  }

  public void init() {
    m_logger.info("Initializing ElevatorSub Subsystem");
    m_elevatorMotor.setPosition(Constants.Elevator.kStartingHeight);
    setPositionMm(Constants.Elevator.kStartingHeight);
  }

  @Override
  public void periodic() {


    if(m_enableAutomation) {
      runHeightControl(true);
    } else {
      runHeightControl(false);
    }

    // Power widget is in setPower method
    SmartDashboard.putNumber("El Enc", getPositionMm()); // Elevator position
    SmartDashboard.putBoolean("El Auto", m_enableAutomation); // True if automation is running
    SmartDashboard.putBoolean("El UpLimit", isAtUpperLimit()); // True if we are at the upper limit
    SmartDashboard.putBoolean("El Set Enc", m_isElevatorEncoderSet); // True once the encoder is set
    SmartDashboard.putBoolean("El RstEnc", encoderResetSwitchHit()); // True if we hit the encoder reset switch

    // If we haven't set the relative encoder's position yet, check if we are at the switch that tells us to do so                                                                                                                                                                                                                                                                                                                                                                                                                          
    if(!m_isElevatorEncoderSet) {
      // Adds a counter to the encoder reset switch so that we don't reset position by accident
      if(encoderResetSwitchHit() && (m_elevatorMotor.get() > 0.0)) {
        m_hitEncoderSwitchCounter++;
      } else {
        m_hitEncoderSwitchCounter = 0;
      }

      // If we hit the reset switch twice in a row, reset encoder
      if(m_hitEncoderSwitchCounter >= 2) {
        m_elevatorMotor2.setPosition(Constants.Elevator.kResetHeight);
        m_isElevatorEncoderSet = true;
      }
    }
  }

  /**
   * Manually set the power of the elevator motor(s).
   * 
   * @param power power value -1.0 to 1.0
   */
  public void setPower(double power) {
    // If lower limit switch is hit and the motor is going down, stop.
    // If we are too close to the lower limit, set max power to a low value
    // If we are too close to the upper limit, set max power to a low value
    // Otherwise, set power normally
    double powerValue = power;
    if(isAtLowerLimit() && power < 0.0) {
      powerValue = 0.0;
      System.out.println("Lower limit hit");
    } else if(isAtUpperLimit() && power > 0.0) {
      powerValue = m_feedforward.calculate(getVelocity()) + m_elevatorPID.calculate(getPositionMm(), m_targetHeight);
      System.out.println("Upper limit hit");
    } else if((getPositionMm() < Constants.Elevator.kSlowDownLowerStageHeight)
        && (power < Constants.Elevator.kSlowDownLowerStagePower)) {
      powerValue = Constants.Elevator.kSlowDownLowerStagePower;
    } else if((getPositionMm() > Constants.Elevator.kSlowDownUpperStageHeight)
        && (power > Constants.Elevator.kSlowDownUpperStagePower)) {
      powerValue = Constants.Elevator.kSlowDownUpperStagePower;
    }

    m_elevatorMotor.set(powerValue);
    SmartDashboard.putNumber("El Power", powerValue); // Elevator power
  }

  /**
   * Sets the current height
   */
  public void setPositionMm(double height) {
    m_elevatorMotor.setPosition(height);
  }

  /**
   * Returns the current height of the elevator
   * 
   * @return position in mm
   */
  public double getPositionMm() {
    return m_elevatorMotor.getPosition().getValueAsDouble();
  }

  /**
   * Returns the current velocity of the elevator
   * 
   * @return velocity in mm per second
   */
  public double getVelocity() {
    return m_elevatorMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Sets the target height that the elevator should move to
   * 
   * @param targetAngle target angle in degrees
   */
  public void setTargetHeight(double targetHeight) {
    if(targetHeight >= Constants.Elevator.kMaxHeight) {
      //Constants.Elevator.kMaxHeight;
    } else if(targetHeight <= Constants.Elevator.kMinHeight) {
      targetHeight = Constants.Elevator.kMinHeight;
    }

    m_targetHeight = targetHeight;
    enableAutomation();
    runHeightControl(false);
  }

  /**
   * Enables closed loop control of elevator height
   */
  public void enableAutomation() {
    m_enableAutomation = true;
  }

  /**
   * Disables closed loop control of elevator height
   */
  public void disableAutomation() {
    m_enableAutomation = false;
  }

  /**
   * Returns if the elevator is at its lower limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  public boolean isAtLowerLimit() {
    return false; // We don't have a lower limit switch
  }

  /**
   * Returns if the elevator is at its upper limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  public boolean isAtUpperLimit() {
    return !m_elevatorUpperLimit.get();
  }

  /**
   * Returns if the elevator is at the encoder reset switch or not
   * 
   * @return true when it's at the switch, false otherwise
   */
  public boolean encoderResetSwitchHit() {
    return !m_encoderResetSwitch.get();
  }


  /**
   * Calculates and sets the current power to apply to the elevator to get to or stay at its target
   * 
   * @param updatePower set to false to update the Feedforward and PID controllers without changing the motor power
   */
  private void runHeightControl(boolean updatePower) {
    double ffPower = m_feedforward.calculate(getVelocity());
    double pidPower = (m_elevatorPID.calculate(getPositionMm(), m_targetHeight));

    if(updatePower) {
      setPower(ffPower + pidPower);
    }
  }

  private boolean isBlocked() {
    /*
     * should stop the elevator if:
     * - the elevator position is lower than the bottom danger zone and the
     * elevator is moving downwards, and also the arm is not close to pointing downwards
     * 
     * - the elevator position is higher than the bottom brace, and if the arm is holding a coral
     * AND facing close to downwards, and the elevator is moving downwards
     * 
     * - the elevator position is higher than the bottom brace but lower than the top brace, and the arm is
     * holding a coral and is facing close to downwards, AND the elevator is moving upwards
     */

    double currentHeight = getPositionMm();
    double armAngle = m_armAngle.get();
    if((m_targetHeight > currentHeight)) {
      // moving upwards
      if((currentHeight > Constants.Elevator.kDangerZoneBraceBottom)
          && (currentHeight < Constants.Elevator.kDangerZoneBraceTop)
          && (armAngle < Constants.Elevator.kDangerZoneArmBraceAngle)) {
        return true;
      }
    }
    return false;

  }

  // private void updateStateMachine() {
  //   double newPower = 0.0;
  //   double currentPosition = getPositionMm();

  //   // Check if there are new control parameters to set
  //   if(m_newControlParameters) {
  //     m_currentControl.state = m_newControl.state;
  //     m_currentControl.mode = m_newControl.mode;
  //     m_currentControl.targetPower = m_newControl.targetPower;
  //     m_currentControl.targetPosition = m_newControl.targetPosition;
  //     m_newControlParameters = false;
  //   }

  //   // Determine what power the mechanism should use based on the current state
  //   switch(m_currentControl.state) {
  //     case IDLE:
  //       // If the state machine is idle, don't supply any power to the mechanism
  //       newPower = 0.0;
  //       break;

  //     case MOVING:
  //       // If the mechanism is moving, check if it has arrived at it's target.
  //       if(isBlocked(currentPosition, m_currentControl.targetPosition)) {
  //         m_blockedPosition = currentPosition;
  //         m_currentControl.state = State.INTERRUPTED;
  //       } else if(isAtTargetHeight()) {
  //         m_currentControl.state = State.HOLDING;
  //       } else {
  //         newPower = calcMovePower(currentPosition, m_currentControl.targetPosition, m_currentControl.targetPower);
  //       }
  //       break;

  //     case HOLDING:
  //       // If the mechanism is at it's target location, apply power to hold it there if necessary
  //       newPower = calcMovePower(currentPosition, m_currentControl.targetPosition, m_currentControl.targetPower);
  //       break;

  //     case INTERRUPTED:
  //       // If the mechanism is no longer blocked, transition to MOVING
  //       if(!isBlocked(currentPosition, m_currentControl.targetPosition)) {
  //         m_currentControl.state = State.MOVING;
  //         // Otherwise, hold this position
  //       } else {
  //         newPower = calcMovePower(currentPosition, m_blockedPosition, m_currentControl.targetPower);
  //       }
  //       break;

  //     default:
  //       m_currentControl.state = State.HOLDING;
  //       break;
  //   }

  //   if(newPower != m_lastPower) {
  //     setPower(newPower);
  //     m_lastPower = newPower;
  //   }

  // }

  /**
   * Returns if the elevator has reached it's target height or not
   * 
   * @return true if at target height, false if not
   */
  public boolean isAtTargetHeight() {
    // If we are within tolerance and our velocity is low, we're at our target
    if((Math.abs(m_targetHeight - getPositionMm()) < Constants.Elevator.kHeightTolerance)
        && (getVelocity() < Constants.Elevator.kAtTargetMaxVelocity)) {
      return true;
    } else {
      return false;
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
    // Save the current encoder values because the tests will reset them
    m_preTestHeight = m_elevatorMotor.getPosition().getValueAsDouble();
    m_preTestHeight2 = m_elevatorMotor.getPosition().getValueAsDouble();

    // Disable any mechanism automation (PID, etc.).  Check periodic()
    disableAutomation();
  }

  /**
   * Puts the motor back into normal opeation mode.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  @Override
  public void testDisableMotorTestMode(int motorId) {
    // Set the encoders to their pre-test values plus the change in position from the test
    m_elevatorMotor.setPosition(m_preTestHeight + m_elevatorMotor.getPosition().getValueAsDouble(), 0.5);
    m_elevatorMotor2.setPosition(m_preTestHeight2 + m_elevatorMotor2.getPosition().getValueAsDouble(), 0.5);

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
        m_elevatorMotor.setPosition(0.0, 0.5); // Set it to 0 and wait up to half a second for it to take effect
        break;
      case 2:
        m_elevatorMotor2.setPosition(0.0, 0.5); // Set it to 0 and wait up to half a second for it to take effect
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

    m_elevatorMotor.set(power);

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
        position = m_elevatorMotor.getPosition().getValueAsDouble(); // This position is affected by the conversion factor
        break;
      case 2:
        position = m_elevatorMotor2.getPosition().getValueAsDouble(); // This position is affected by the conversion factor
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
        current = m_elevatorMotor.getStatorCurrent().getValueAsDouble();
        break;
      case 2:
        current = m_elevatorMotor2.getStatorCurrent().getValueAsDouble();
        break;
      default:
        // Return an invalid value
        current = -1.0;
        break;
    }

    return current;
  }
}
