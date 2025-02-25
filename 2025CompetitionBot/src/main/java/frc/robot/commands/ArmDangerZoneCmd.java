// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ArmDangerZoneCmd extends Command {
  private final ElevatorSub m_elevatorSub;
  private final ArmSub m_armSub;
  public double eHeight;
  public double aHeight;

  /** Creates a new ArmDangerZoneCmd. */
  public ArmDangerZoneCmd(ElevatorSub elevatorSub, ArmSub armSub) {
    m_elevatorSub = elevatorSub;
    m_armSub = armSub;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    eHeight = m_elevatorSub.getPositionMm();
    aHeight = m_armSub.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(eHeight <= Constants.DangerZones.kElevatorDangerZone1) {
      if(aHeight <= Constants.DangerZones.kArmDangerZoneRange1
          && aHeight >= Constants.DangerZones.kArmDangerZoneRange2) {
        m_elevatorSub.setPower(0.0);
      }
    } else if(Constants.DangerZones.kElevatorDangerZoneRange1 <= eHeight
        && eHeight <= Constants.DangerZones.kElevatorDangerZoneRange2) { // values in mm, PLEASE CHANGE THEM NOW
      if(aHeight <= Constants.DangerZones.kArmDangerZone1) {
        m_elevatorSub.setPower(0.0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
