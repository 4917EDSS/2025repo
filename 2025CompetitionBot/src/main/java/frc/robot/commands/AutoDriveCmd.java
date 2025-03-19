// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.utils.RobotStatus;
import frc.robot.utils.RobotStatus.ReefPosition;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class AutoDriveCmd extends Command {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final VisionSub m_visionSub;
  private final SwerveRequest.RobotCentric autoDrive = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  Pose2d m_apriltagPos;
  double lrDist;
  double fbDist;
  int counter;
  double lrOffset;
  double fbOffset;
  double fbEnd;
  boolean useOffset;
  private final DrivetrainSub m_drivetrainSub;

  /** Creates a new AutoDriveCmd. */
  public AutoDriveCmd(VisionSub visionSub, DrivetrainSub drivetrainSub, boolean useOffset) {
    m_visionSub = visionSub;
    m_drivetrainSub = drivetrainSub;
    this.useOffset = useOffset;
    addRequirements(drivetrainSub);// Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(RobotStatus.LastReefPosition().equals(ReefPosition.kL2L3Algae)
        || RobotStatus.LastReefPosition().equals(ReefPosition.kL3L4Algae)) {
      fbOffset = -0.1;
      fbEnd = 0.0;
    } else {
      fbOffset = 0.15;
      fbEnd = 0.49;
      if(RobotStatus.LastReefPosition().equals(RobotStatus.ReefPosition.kL4)) {
        fbEnd += 0.0127; //This is half an inch in meters
      }
    }

    if(useOffset) {
      if(RobotStatus.isLeft()) {
        lrOffset = 0.17;
      } else {
        lrOffset = -0.17;
      }
    } else {
      lrOffset = 0;
    }
    // Use open-loop control for drive motors
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_apriltagPos = m_visionSub.getTagPose2d();
    // check if angle is positive or negative
    lrDist = m_apriltagPos.getX() + lrOffset;
    fbDist = m_apriltagPos.getY() + fbOffset;
    double totalDist = Math.sqrt((lrDist * lrDist) + (fbDist * fbDist));
    double xPower = lrDist / totalDist;
    double yPower = fbDist / totalDist;
    double slowDown;
    if(fbDist > -1.5) {
      slowDown = 6;
    } else {
      slowDown = 2;
    }
    m_drivetrainSub.setControl(
        autoDrive.withVelocityX(-yPower * MaxSpeed / slowDown).withVelocityY(xPower * MaxSpeed / 1.5)
            .withRotationalRate(m_visionSub.getRobotRotation() / ((lrDist * 15) + 15) * MaxAngularRate * 0.20));

    if(m_visionSub.getTv() == 0) {
      counter++;
    } else {
      counter = 0;
    }
    // rotate in desired direction until angle is 0
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.applyRequest(() -> brake);
    m_drivetrainSub.setControl(autoDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(Math.abs(fbDist) < fbEnd && Math.abs(lrDist) < 0.025) {

      return true;
    }
    if(counter >= 25) {
      return true;
    }
    return false;
  }
}
