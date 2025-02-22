// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.utils.LimelightHelpers;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class DriveToPosRobotRelativeCmd extends Command {
  private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);;
  private final DrivetrainSub m_drivetrainSub;
  double targetX;
  double targetY;
  double currentX;
  double currentY;
  double power;

  /** Creates a new DriveToPosobotRelativeCmd. */
  public DriveToPosRobotRelativeCmd(double targetX, double targetY, double targetRot, Double power,
      DrivetrainSub drivetrainSub) {
    m_drivetrainSub = drivetrainSub;
    this.targetX = targetX;
    this.targetY = targetY;
    this.power = power;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d[] modulePosArray = m_drivetrainSub.getModuleLocations();

    double totalDist = Math.sqrt((targetX * targetX) + (targetY * targetY));
    double xPower = targetX / totalDist;
    double yPower = targetY / totalDist;

    m_drivetrainSub.setControl(
        driveRobotCentric.withVelocityX(xPower * Constants.DriveTrain.kMaxSpeed * power)
            .withVelocityY(yPower * Constants.DriveTrain.kMaxSpeed * power)
            .withRotationalRate(Constants.DriveTrain.kMaxAngularRate * 0.5));
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
