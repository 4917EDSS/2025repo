// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.comands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeoTestSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class PivotToAngleCmd extends Command {
  private final double m_angle;
  private final NeoTestSub m_neotestSub;

  /** Creates a new PivotToAngleCmd. */
  public PivotToAngleCmd(double angle, NeoTestSub neoTestSub) {
    m_angle = angle;
    m_neotestSub = neoTestSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(neoTestSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // put thing for positive or negative direction
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((m_neotestSub.getAngle() - m_angle) < 0) {
      m_neotestSub.setPower(0.5);
    } else {
      m_neotestSub.setPower(-0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_neotestSub.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(m_neotestSub.getAngle() - m_angle) < 0.5) {
      return true;
    } else {
      return false;
    }
  }
}
