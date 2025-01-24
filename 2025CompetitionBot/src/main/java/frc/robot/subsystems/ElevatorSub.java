// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSub extends SubsystemBase {
  private final TalonFX m_elevatorMotor = new TalonFX(Constants.CanIds.kElevatorMotor);
  private final TalonFX m_elevatorMotor2 = new TalonFX(Constants.CanIds.kElevatorMotor2);
  private final DigitalInput m_elevatorLowerLimit = new DigitalInput(Constants.DioIds.kElevatorLowerLimit);
  private final DigitalInput m_elevatorUpperLimit = new DigitalInput(Constants.DioIds.kElevatorUpperLimit);

  // TODO:  Either finish off the proper units implementation or get rid of it
  private static final PerUnit<DistanceUnit, AngleUnit> MetersPerDegrees = Meters.per(Degrees);
  private static final Per<DistanceUnit, AngleUnit> kConversionToHeight = MetersPerDegrees.ofNative(0.000416);

  // TOOD:  Need a feed-forward controller here
  private final PIDController m_elevatorPID = new PIDController(0.002, 0.0, 0.0);

  private double m_targetHeight = 0.0;
  private boolean m_areWeTryingToHold = true;
  private int m_hitLimitSwitchCounter = 0;


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
    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFxConfiguarator.apply(outputConfigs);
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Invert = Clockwise
    talonFxConfiguarator2.apply(outputConfigs);

    resetPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_areWeTryingToHold) {
      runHeightControl(false);
    }
    if(isAtLowerLimit() && Math.abs(getPosition()) > 5.0) {
      m_hitLimitSwitchCounter++;
    } else {
      m_hitLimitSwitchCounter = 0;
    }

    if(m_hitLimitSwitchCounter >= 5) {
      resetPosition();
      m_hitLimitSwitchCounter = 0;
    }
  }

  /**
   * Manually set the power of the elevator motor(s).
   * 
   * @param power power value -1.0 to 1.0
   */
  public void setPower(double power) {
    m_elevatorMotor.set(power);
  }

  /**
   * Sets the current height as the zero height
   */
  public void resetPosition() {
    m_elevatorMotor.setPosition(0);
    m_elevatorMotor2.setPosition(0);
  }

  /**
   * Returns the current angular position of the arm
   * 
   * @return position in degrees
   */
  public double getPosition() {
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
    m_areWeTryingToHold = true;
    runHeightControl(true);
  }

  /**
   * Returns if the elevator is at its lower limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  public boolean isAtLowerLimit() {
    return m_elevatorLowerLimit.get();
  }

  /**
   * Returns if the elevator is at its upper limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  public boolean isAtUpperLimit() {
    return m_elevatorUpperLimit.get();
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
   * @param justCalculate set to true to update the Feedforward and PID controllers without changing the motor power
   */
  // TODO:  This should not be public.  Should be able to use a flag to turn it off instead.
  public void runHeightControl(boolean justCalculate) {
    // TODO: Create and configure PID and Feedforward controllers
    double pidPower = m_elevatorPID.calculate(getPosition(), m_targetHeight);
    double fedPower = 0;//m_pivotFeedforward.calculate(Math.toRadians(getPivotAngle() - 90.0), pidPower); // Feed forward expects 0 degrees as horizontal

    if(!justCalculate) {
      setPower(pidPower + fedPower);
    }
  }
}
