// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class AutoDriveCmd extends Command {
  private final VisionSub m_visionSub;
  SwerveRequest.FieldCentric autoDrive;
  Pose2d m_apriltagPos;
  double xDist;
  double yDist;
  int counter;

  /** Creates a new AutoDriveCmd. */
  public AutoDriveCmd(VisionSub visionSub) {
    m_visionSub = visionSub;
    addRequirements(visionSub);// Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoDrive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_apriltagPos = m_visionSub.getTagPose2d();
    //check if angle is positive or negative


    if(m_apriltagPos.getRotation().getRadians() > 0.05 && m_apriltagPos.getRotation().getRadians() < -0.05) {
      if(m_apriltagPos.getRotation().getRadians() > 0) {
        RobotContainer.m_drivetrainSub
            .applyRequest(() -> autoDrive.withRotationalRate(RotationsPerSecond.of(-0.75).in(RadiansPerSecond)));
      } else {
        RobotContainer.m_drivetrainSub
            .applyRequest(() -> autoDrive.withRotationalRate(RotationsPerSecond.of(0.75).in(RadiansPerSecond)));
      }
    } else {
      xDist = m_apriltagPos.getX();
      yDist = m_apriltagPos.getY();
      double totalDist = Math.sqrt((xDist * xDist) + (yDist * yDist));
      xDist /= totalDist;
      yDist /= totalDist;
      RobotContainer.m_drivetrainSub.applyRequest(() -> autoDrive.withVelocityX(xDist).withVelocityY(yDist));

    }
    if(m_visionSub.getTv() == false) {
      counter++;
    } else {
      counter = 0;
    }
    //rotate in desired direction until angle is 0

    //calculate using math described earlier (ratios and stuff)
    //call movement code with desired velocities
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrainSub
        .applyRequest(() -> autoDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(xDist < 0.05 && xDist > -0.05 && yDist < 0.45 && yDist > .3) {
      return true;
    }
    if(counter >= 25)
      return true;
    return false;
  }
}
