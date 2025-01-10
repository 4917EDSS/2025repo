// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KrakenTestSub extends SubsystemBase {
  private int count = 10;
  private final TalonFX m_krakenMotor = new TalonFX(Constants.CanIds.kKrakenId);

  /** Creates a new KrakenTestSub. */
  public KrakenTestSub() {
    TalonFXConfigurator talonFxConfiguarator = m_krakenMotor.getConfigurator(); // Need this to change anything

    // This is how you set a current limit inside the motor (vs on the input power supply)
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 3; // Limit in Amps
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFxConfiguarator.apply(limitConfigs);

    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.04; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Coast;//.Brake;
    talonFxConfiguarator.apply(outputConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double power) {
    if(--count == 0) {
      count = 10;
      System.out.println("Kraken " + power);
    }
    m_krakenMotor.set(power);
  }
}
