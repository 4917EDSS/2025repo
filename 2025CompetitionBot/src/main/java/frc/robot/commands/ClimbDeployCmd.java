// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ClimbDeployCmd extends Command {
  ClimbSub m_climbSub;

  /** Creates a new ClimbDeployCmd. */
  public ClimbDeployCmd(ClimbSub climbSub) {
    m_climbSub = climbSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbSub.setPower(Constants.Climb.kClimbMotorPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSub.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climbSub.isAtOutLimit();
  }
}
