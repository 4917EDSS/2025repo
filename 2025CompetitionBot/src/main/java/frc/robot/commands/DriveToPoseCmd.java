// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DrivetrainSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class DriveToPoseCmd extends Command {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(2.08 / 2.0).in(RadiansPerSecond);

  DrivetrainSub m_drivetrainSub;
  Pose2d m_targetPose;
  Pose2d m_currentPose;
  Transform2d m_error;
  Transform2d m_prevError;


  private final SwerveRequest.FieldCentric backDrive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.Velocity).withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  Double m_driveP = 5.0; //TODO: Get proper value
  Double m_driveI = 0.0; //TODO: Get proper value
  Double m_driveD = 0.00; //TODO: Get proper value
  Double m_rotP = 0.1; //TODO: Get proper value
  Double m_rotI = 0.0; //TODO: Get proper value
  Double m_rotD = 0.00; //TODO: Get proper value


  /** Creates a new DriveToPoseCmd. */
  public DriveToPoseCmd(Pose2d targetPose, DrivetrainSub drivetrainSub) {
    m_targetPose = targetPose;
    m_drivetrainSub = drivetrainSub;
    addRequirements(m_drivetrainSub);
    // Use addRequirements() here to declare subsystem.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_error = new Transform2d();
    m_prevError = new Transform2d();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_currentPose = m_drivetrainSub.getPose();
    m_prevError = m_error;

    // Minus and relative seem to do something completely different than expected
    m_error = new Transform2d(m_targetPose.getX() - m_currentPose.getX(), m_targetPose.getY() - m_currentPose.getY(),
        m_targetPose.getRotation().minus(m_currentPose.getRotation()));
    //P
    Translation2d outputDrivePower = m_error.getTranslation().times(m_driveP);
    double outputRotPower = m_error.getRotation().getDegrees() * m_rotP;

    Double driveNormalizationValue = 1.0;
    Double rotNormalizationValue = 1.0;
    //D
    // outputDrivePower = outputDrivePower.plus((m_error.minus(m_prevError)).getTranslation().times(m_driveD / 0.02));//0.02 is the amount of time between periodic calls, which is why it is used as the change in time
    // outputRotPower += ((m_error.minus(m_prevError)).getRotation().getDegrees()) * m_rotD / 0.02;

    // Double totalSpeed = Math.sqrt(Math.pow(outputDrivePower.getX(), 2) + Math.pow(outputDrivePower.getY(), 2));
    // if(totalSpeed > MaxSpeed) {
    //   driveNormalizationValue = MaxSpeed / totalSpeed;
    // }

    // if(outputRotPower > MaxAngularRate) {
    //   rotNormalizationValue = MaxAngularRate / outputRotPower;
    // }
    if(outputDrivePower.getNorm() < 0.1) {
      outputDrivePower = outputDrivePower.times((0.1 / outputDrivePower.getNorm()));
    }

    // I believe setControl is just a less confusing version of applyRequest.
    m_drivetrainSub.setControl(backDrive.withVelocityX(outputDrivePower.getX())
        .withVelocityY(outputDrivePower.getY())
        .withRotationalRate(outputRotPower));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.setControl(backDrive.withVelocityX(0.0).withVelocityY(0.0).withVelocityY(0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_error.getTranslation().getNorm() < 0.02 && Math.abs(m_error.getRotation().getDegrees()) < 2) {
      return true;
    }
    return false;
  }
}
