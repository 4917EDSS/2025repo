// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.generated.TunerConstants;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class SimpleAuto extends Command {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final SwerveRequest.RobotCentric autoDrive = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  Pose2d m_apriltagPos;
  double xDist;
  double yDist;
  int counter;
  double offset;
  private final DrivetrainSub m_drivetrainSub;

  /** Creates a new SimpleAuto. */

  public SimpleAuto(DrivetrainSub drivetrainSub) {
    m_drivetrainSub = drivetrainSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    counter += 1;
    m_drivetrainSub.setControl(
        autoDrive.withVelocityX(0.1 / MaxSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(counter >= 20) {
      return true;
    } else {
      return false;
    }
  }


}
