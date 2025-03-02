// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoseCmd extends Command {
  DrivetrainSub m_drivetrainSub;
  Translation2d m_targetPos;
  Translation2d m_currentPos;
  Translation2d m_error;
  Translation2d m_prevError;
  Double m_drivePowerX;
  Double m_drivePowerY;
  Double m_rotPower;

  private final SwerveRequest.FieldCentric backDrive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  Double m_driveP = 0.01; //TODO: Get proper value
  Double m_driveI = 0.0; //TODO: Get proper value
  Double m_driveD = 0.01; //TODO: Get proper value
  Double m_rotP = 0.01; //TODO: Get proper value
  Double m_rotI = 0.0; //TODO: Get proper value
  Double m_rotD = 0.01; //TODO: Get proper value
    /** Creates a new DriveToPoseCmd. */
  public DriveToPoseCmd(Translation2d targetPos, DrivetrainSub drivetrainSub) {
    m_targetPos = targetPos;
    m_drivetrainSub = drivetrainSub;
    addRequirements(m_drivetrainSub);
    // Use addRequirements() here to declare subsystem.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentPos = m_drivetrainSub.getPose().getTranslation();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_prevError = m_error;
    m_error = m_targetPos.minus(m_currentPos);
    //P
    m_drivePowerX += m_error.times(m_driveP).getX();
    m_drivePowerY += m_error.times(m_driveP).getY();
    m_rotPower += m_error.times(m_driveP).getAngle().getDegrees();

    //D (I did this out of my own curiocity, was wondering if it was correct)
    m_drivePowerX += (m_prevError.minus(m_error).getX())/0.02;//0.02 is the amount of time between periodic calls, which is why it is used as the change in time
    m_drivePowerX += (m_prevError.minus(m_error).getY())/0.02;
    m_rotPower += (m_prevError.minus(m_error).getAngle().getDegrees())/0.02;

    m_drivetrainSub.applyRequest(() -> backDrive.withVelocityX(m_drivePowerX).withVelocityY(m_drivePowerY).withRotationalRate(m_rotPower));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(m_currentPos.getDistance(m_targetPos))<0.02){
      return true;
    }
    return false;
  }
}
