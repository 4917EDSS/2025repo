// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.TestableSubsystem;

public class ElevatorSub extends TestableSubsystem {
  private final TalonFX m_elevatorMotor = new TalonFX(Constants.CanIds.kElevatorMotor);
  private final TalonFX m_elevatorMotor2 = new TalonFX(Constants.CanIds.kElevatorMotor2);
  private final DigitalInput m_elevatorLowerLimit = new DigitalInput(Constants.DioIds.kElevatorLowerLimit);
  private final DigitalInput m_elevatorUpperLimit = new DigitalInput(Constants.DioIds.kElevatorUpperLimit);
  private final DigitalInput m_elevatorEncoderResetSwitch =
      new DigitalInput(Constants.DioIds.kElevatorEncoderResetSwitch);
  private final DigitalInput m_elevatorStageTwoLimit = new DigitalInput(Constants.DioIds.kElevatorStageTwoLimit);

  // TODO:  Either finish off the proper units implementation or get rid of it

  // TODO:  Need a feed-forward controller here
  private final PIDController m_elevatorPID = new PIDController(0.002, 0.0, 0.0);

  private double m_targetHeight = 0.0;
  private boolean m_enableAutomation = false;
  private int m_hitLimitSwitchCounter = 0;
  public boolean m_elevatorMotorDirection = true;
  private boolean m_isElevatorEncoderSet = false;
  private int m_hitEncoderSwitchCounter = 0;


  /** Creates a new ElevatorSub. */
  public ElevatorSub() {
    TalonFXConfigurator talonFxConfiguarator = m_elevatorMotor.getConfigurator();
    TalonFXConfigurator talonFxConfiguarator2 = m_elevatorMotor2.getConfigurator();

    // This is how you set a current limit inside the motor (vs on the input power supply)
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 40; // Limit in Amps
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

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = Constants.Elevator.kRotationsToMm;

    m_elevatorMotor.getConfigurator().apply(config);

    //final DutyCycleOut m_dutyCycle = new DutyCycleOut(0.0);
    // m_elevatorMotor.setControl(m_dutyCycle.withOutput(0.5)
    //.withLimitForwardMotion(m_elevatorLowerLimit.get())
    //.withLimitReverseMotion(m_elevatorUpperLimit.get()));

    setPositionMm(Constants.Elevator.kStartingHeight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_enableAutomation) {
      runHeightControl(true);
    } else {
      runHeightControl(false);
    }

    SmartDashboard.putNumber("El Enc", getPositionMm());
    SmartDashboard.putBoolean("El Auto", m_enableAutomation);
    SmartDashboard.putBoolean("El UpLimit", isAtUpperLimit());

    // Reset encoder if we hit the limit switch and the position doesn't read close to zero
    if(isAtLowerLimit() && Math.abs(getPositionMm()) > 5.0) {
      m_hitLimitSwitchCounter++;
    } else {
      m_hitLimitSwitchCounter = 0;
    }

    // Adds a counter to the encoder reset switch so that we don't reset position by accident
    if(encoderResetSwitchHit()) {
      m_hitEncoderSwitchCounter++;
    } else {
      m_hitEncoderSwitchCounter = 0;
    }

    //Resets encoder when the limit switch is hit
    // if(m_hitLimitSwitchCounter >= 2) {
    //   setPositionMm(0);
    //   m_hitLimitSwitchCounter = 0;
    // }

    // Sets encoder position when Encoder Reset Switch is hit
    if((m_hitEncoderSwitchCounter >= 2) && (m_elevatorMotor.get() > 0.0) && (!m_isElevatorEncoderSet)) {
      m_elevatorMotor.setPosition(445);
      m_elevatorMotor2.setPosition(445);
      m_isElevatorEncoderSet = true;
    } //else if((encoderResetSwitchHit()) && (m_elevatorMotor.get() < 0.0)) {
    //   m_elevatorMotor.setPosition(10);
    //   m_elevatorMotor2.setPosition(10);
    // }

    // Sets encoder position when Stage 2 Limit is hit
    // if((isAtStageTwoLimit()) && (m_elevatorMotor.get() > 0.0)) {
    //   m_elevatorMotor.setPosition(1000);
    //   m_elevatorMotor2.setPosition(1000);
    //   System.out.println("Stage 2 limit hit");
    // } else if((isAtStageTwoLimit()) && (m_elevatorMotor.get() < 0.0)) {
    //   m_elevatorMotor.setPosition(1005);
    //   m_elevatorMotor2.setPosition(1005);
    //   System.out.println("Stage 2 limit hit");
    // }

    //Sets encoder position when Upper Limit is hit
    // if(isAtUpperLimit()) {
    //   m_elevatorMotor.setPosition(2000);
    // }
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
      powerValue = 0.0;
      System.out.println("Upper limit hit");
    } else if((getPositionMm() < Constants.Elevator.kSlowDownLowerStageHeight)
        && (power < Constants.Elevator.kSlowDownLowerStagePower)) {
      powerValue = Constants.Elevator.kSlowDownLowerStagePower;
    } else if((getPositionMm() > Constants.Elevator.kSlowDownUpperStageHeight)
        && (power > Constants.Elevator.kSlowDownUpperStagePower)) {
      powerValue = Constants.Elevator.kSlowDownUpperStagePower;
    }
    m_elevatorMotor.set(powerValue);
    //check elevator direction
    if(powerValue > 0) {
      m_elevatorMotorDirection = true;
    } else {
      m_elevatorMotorDirection = false;
    }
    SmartDashboard.putNumber("El Power", powerValue);
    // System.out.println("Current power is " + powerValue);
    // System.out.println("Position is " + getPositionMm());
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
   * @return velocity in degrees per second
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
      targetHeight = Constants.Elevator.kMaxHeight;
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
    return !m_elevatorLowerLimit.get();
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
    return m_elevatorEncoderResetSwitch.get();
  }

  /**
   * Returns if the elevator is at the stage 2 limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  public boolean isAtStageTwoLimit() {
    return m_elevatorStageTwoLimit.get();
  }

  /**
   * Returns how much current the motor is currently drawing
   * 
   * @return current in amps or -1.0 if motor can't measure current
   */
  public double getElectricalCurrent() {
    // TODO:  Need to figure this out for multiple motors
    return -1.0;
  }

  public void testElevatorMotor(double power) {
    m_elevatorMotor.set(power);
    m_elevatorMotor2.set(power);
  }


  /**
   * Calculates and sets the current power to apply to the elevator to get to or stay at its target
   * 
   * @param updatePower set to false to update the Feedforward and PID controllers without changing the motor power
   */
  private void runHeightControl(boolean updatePower) {
    //TODO: Create and configure PID and Feedforward controllers
    double pidPower = (m_elevatorPID.calculate(getPositionMm(), m_targetHeight));
    if(pidPower > .5) {
      pidPower = .5;
    }
    // if(m_targetHeight < getPositionMm()) {
    //   pidPower = 0;
    // }
    double fedPower = 0.01;

    if(updatePower) {
      setPower(pidPower + fedPower);
    }

    // if(updatePower) {
    //   double delta = m_targetHeight - getPositionMm();
    //   if(Math.abs(delta) < 10) {
    //     setPower(0);
    //   } else if(delta > 0) {
    //     setPower(0.1);
    //   } else {
    //     setPower(-0.1);
    //   }
    // }
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
    // Disable any mechanism automation (PID, etc.).  Check periodic()
  }

  /**
   * Puts the motor back into normal opeation mode.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  @Override
  public void testDisableMotorTestMode(int motorId) {
    enableAutomation();
    // Re-ensable any mechanism automation
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
    switch(motorId) {
      case 1:
        m_elevatorMotor.set(power);
        break;
      case 2:
        m_elevatorMotor2.set(power);
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

  public boolean isElevatorAtTargetHeight() {
    if(m_targetHeight <= getPositionMm() + Constants.Elevator.kHeightTolerance
        && m_targetHeight >= getPositionMm() - Constants.Elevator.kHeightTolerance) { //Add tolerance
      return true;
    } else {
      return false;
    }
  }
}
