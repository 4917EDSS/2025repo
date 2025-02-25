// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PowerSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(PowerSub.class.getName());

  PowerDistribution m_powerDistributionModule = new PowerDistribution(1, ModuleType.kRev);

  /** Creates a new PowerSub. */
  public PowerSub() {
    // Get the voltage going into the PDP, in Volts.
    // The PDP returns the voltage in increments of 0.05 Volts.
    SmartDashboard.putNumber("Voltage", m_powerDistributionModule.getVoltage());
    // Retrieves the temperature of the PDP, in degrees Celsius.
    SmartDashboard.putNumber("Temperature", m_powerDistributionModule.getTemperature());
    // Get the total current of all channels.
    SmartDashboard.putNumber("Current", m_powerDistributionModule.getTotalCurrent());
    // Get the total power of all channels.
    // Power is the bus voltage multiplied by the current with the units Watts.
    SmartDashboard.putNumber("Power", m_powerDistributionModule.getTotalPower());
    // Get the total energy of all channels.
    // Energy is the power summed over time with units Joules.
    SmartDashboard.putNumber("Energy", m_powerDistributionModule.getTotalEnergy());

    //BREAKERS START HERE
    // Get the current in Amperes.
    // The PDP returns the current in increments of 0.125A.
    // At low currents the current readings tend to be less accurate.
    SmartDashboard.putNumber("DFR", Constants.Breakers.kDriveMotorFrontRight);
    SmartDashboard.putNumber("DFL", Constants.Breakers.kDriveMotorFrontLeft);
    SmartDashboard.putNumber("DBR", Constants.Breakers.kDriveMotorBackRight);
    SmartDashboard.putNumber("DBL", Constants.Breakers.kDriveMotorBackLeft);

    SmartDashboard.putNumber("SFR", Constants.Breakers.kSteeringMotorFR);
    SmartDashboard.putNumber("SFL", Constants.Breakers.kSteeringMotorFL);
    SmartDashboard.putNumber("SBR", Constants.Breakers.kSteeringMotorBR);
    SmartDashboard.putNumber("SBL", Constants.Breakers.kSteeringMotorBL);

    SmartDashboard.putNumber("CML", Constants.Breakers.kClimbMotorL);
    SmartDashboard.putNumber("CMR", Constants.Breakers.kClimbMotorR);

    SmartDashboard.putNumber("LF", Constants.Breakers.kLowerFeeder);
    SmartDashboard.putNumber("UF", Constants.Breakers.kUpperFeeder);

    SmartDashboard.putNumber("Pivot", Constants.Breakers.kPivot);
    SmartDashboard.putNumber("FlywheelL", Constants.Breakers.kFlywheelL);
    SmartDashboard.putNumber("FlywheelR", Constants.Breakers.kFlywheelR);
    SmartDashboard.putNumber("Intake Rollers", Constants.Breakers.kIntakeRollers);

    SmartDashboard.putNumber("Lime Up", Constants.Breakers.kLimelightUp);
    SmartDashboard.putNumber("Lime Down", Constants.Breakers.kLimelightDown);
    SmartDashboard.putNumber("Cancoders", Constants.Breakers.kCancoders);
    SmartDashboard.putNumber("VRM", Constants.Breakers.kVRM);
    SmartDashboard.putNumber("RoboRio", Constants.Breakers.kRoboRio);
    SmartDashboard.putNumber("Arduino", Constants.Breakers.kArduino);
    SmartDashboard.putNumber("Radio", Constants.Breakers.kRadio);
  }

  public void init() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
