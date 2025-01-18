// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestableSubsystem extends SubsystemBase {
  /** Creates a new MotorSub. */
  public TestableSubsystem() {}

  public double getPosition() {
    return 0;
  }

  public double getVelocity() {
    return 0;
  }

  public double getAcceleration() {
    return 0;
  }

  public double getAmps() {
    return 0;
  }

  public void resetPosition() {

  }

  public void runMotor(double power) {}
}
