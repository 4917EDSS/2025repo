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
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.ImmutablePer;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSub extends SubsystemBase {
  private final TalonFX m_elevatorMotor = new TalonFX(Constants.CanIds.kElevatorMotor);

  private static final PerUnit<DistanceUnit, AngleUnit> MetersPerDegrees = Meters.per(Degrees);
  private static final Per<DistanceUnit, AngleUnit> kConversionToHeight = MetersPerDegrees.ofNative(0.000416);

  /** Creates a new ElevatorSub. */
  public ElevatorSub() {
    TalonFXConfigurator talonFxConfiguarator = m_elevatorMotor.getConfigurator(); // Need this to change anything

    // This is how you set a current limit inside the motor (vs on the input power supply)
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 40; // Limit in Amps
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFxConfiguarator.apply(limitConfigs);

    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFxConfiguarator.apply(outputConfigs);

    m_elevatorMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setElevatorMotor(double power) {
    m_elevatorMotor.set(power);
  }

  public Distance getElevatorPosition() {
    // TODO: We need to fix this properly, the conversion is not working
    return (Distance) kConversionToHeight.timesDivisor(m_elevatorMotor.getPosition().getValue());
  }
}
