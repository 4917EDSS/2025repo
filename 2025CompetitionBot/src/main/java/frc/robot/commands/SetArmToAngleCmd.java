// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetArmToAngleCmd extends Command {
  double m_targetAngle;
  ArmSub m_armSub;
  /** Creates a new SetArmToAngleCmd. */
  public SetArmToAngleCmd(ArmSub armSub, double targetAngle) {
    m_armSub = armSub;
    m_targetAngle = targetAngle;
    addRequirements(m_armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO - ask for the arm to go to m_targetAngle
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO - check if we are done via ArmSub's isAtTargetAngle
    return false;
  }
}
