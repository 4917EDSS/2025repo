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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSub extends SubsystemBase {
  private final TalonFX m_elevatorMotor = new TalonFX(Constants.CanIds.kElevatorMotor);
  private final TalonFX m_elevatorMotor2 = new TalonFX(Constants.CanIds.kElevatorMotor2);
  private final DigitalInput m_elevatorLowerLimit = new DigitalInput(Constants.DioIds.kElevatorLowerLimit);
  private final DigitalInput m_elevatorUpperLimit = new DigitalInput(Constants.DioIds.kElevatorUpperLimit);

  private static final PerUnit<DistanceUnit, AngleUnit> MetersPerDegrees = Meters.per(Degrees);
  private static final Per<DistanceUnit, AngleUnit> kConversionToHeight = MetersPerDegrees.ofNative(0.000416);

  private double m_targetHeight = 0.0;
  private boolean m_areWeTryingToHold = true;
  private int m_hitLimitCounter = 0;

  private final PIDController m_elevatorPID = new PIDController(0.002, 0.0, 0.0);

  /** Creates a new ElevatorSub. */
  public ElevatorSub() {
    TalonFXConfigurator talonFxConfiguarator = m_elevatorMotor.getConfigurator(); // Need this to change anything
    TalonFXConfigurator talonFxConfiguarator2 = m_elevatorMotor2.getConfigurator(); // Need this to change anything

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

    resetEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_areWeTryingToHold) {
      runHeightControl(false);
    }
    if(isAtBottom() && Math.abs(getHeight()) > 5.0) {
      m_hitLimitCounter++;
    } else {
      m_hitLimitCounter = 0;
    }

    if(m_hitLimitCounter >= 5) {
      resetEncoder();
      m_hitLimitCounter = 0;
    }
  }

  public void setTargetHeight(double height) {
    if(height >= Constants.Elevator.kMaxHeight) {
      height = Constants.Elevator.kMaxHeight;
    } else if(height <= Constants.Elevator.kMinHeight) {
      height = Constants.Elevator.kMinHeight;
    }

    m_targetHeight = height;
    m_areWeTryingToHold = true;
    runHeightControl(true);
  }

  public void setElevatorMotor(double power) {
    m_elevatorMotor.set(power);
  }

  public Double getHeight() {
    // TODO: We need to fix this properly, the conversion is not working
    return m_elevatorMotor.getPosition().getValueAsDouble();
  }

  public void resetEncoder() {
    m_elevatorMotor.setPosition(0);
    m_elevatorMotor2.setPosition(0);
  }

  public boolean isAtBottom() {
    return m_elevatorLowerLimit.get();
  }

  public boolean isAtTop() {
    return m_elevatorUpperLimit.get();
  }

  public void runHeightControl(boolean justCalculate) {
    // TODO: Create and configure PID and Feedforward controllers
    double pidPower = m_elevatorPID.calculate(getHeight(), m_targetHeight);
    double fedPower = 0;//m_pivotFeedforward.calculate(Math.toRadians(getPivotAngle() - 90.0), pidPower); // Feed forward expects 0 degrees as horizontal

    if(!justCalculate) {
      setElevatorMotor(pidPower + fedPower);
    }
  }

  public void testElevatorMotor(double power) {
    m_elevatorMotor.set(power);
    m_elevatorMotor2.set(power);
  }

}
