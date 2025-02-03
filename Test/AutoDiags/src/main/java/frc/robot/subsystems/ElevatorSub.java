// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.utils.TestableSubsystem;

public class ElevatorSub extends TestableSubsystem {
  private final TalonFX m_elevatorMotor1 = new TalonFX(Constants.CandIds.kElevatorMotor1);
  private final TalonFX m_elevatorMotor2 = new TalonFX(Constants.CandIds.kElevatorMotor2);


  /** Creates a new ElevatorSub. */
  public ElevatorSub() {
    // Set up motor configuration, including encoder conversion factor
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power) {
    // Simplistic way of controlling the motors.  Should use the motor configuration features.
    m_elevatorMotor1.set(power);
    m_elevatorMotor2.set(-power);
  }

  public void getPositionMm() {
    // Use motor 1's encoder for position
    m_elevatorMotor1.getPosition().getValueAsDouble();
  }


  //////////////////// Methods used for automated testing ////////////////////
  /**
   * Makes the motor ready for testing. This includes disabling any automation that uses this
   * motor
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  public void testEnableMotorTestMode(int motorId) {
    // Disable any mechanism automation (PID, etc.).  Check periodic()
  }

  /**
   * Puts the motor back into normal opeation mode.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  public void testDisableMotorTestMode(int motorId) {
    // Re-ensable any mechanism automation
  }

  /**
   * Resets the motor's encoder such that it reads zero
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  public void testResetMotorPosition(int motorId) {
    switch(motorId) {
      case 1:
        m_elevatorMotor1.setPosition(0.0, 0.5); // Set it to 0 and wait up to half a second for it to take effect
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
  public void testSetMotorPower(int motorId, double power) {
    switch(motorId) {
      case 1:
        m_elevatorMotor1.set(power);
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
  public double testGetMotorPosition(int motorId) {
    double position = 0.0;

    switch(motorId) {
      case 1:
        position = m_elevatorMotor1.getPosition().getValueAsDouble(); // This position is affected by the conversion factor
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
  public double testGetMotorAmps(int motorId) {
    double current = 0.0;

    switch(motorId) {
      case 1:
        current = m_elevatorMotor1.getStatorCurrent().getValueAsDouble();
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
