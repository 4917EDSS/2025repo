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
  double xDist;
  double yDist;
  int counter;
  double offset;
  private final DrivetrainSub m_drivetrainSub;

  /** Creates a new AutoDriveCmd. */
  public AutoDriveCmd(VisionSub visionSub, DrivetrainSub drivetrainSub) {
    m_visionSub = visionSub;
    m_drivetrainSub = drivetrainSub;
    addRequirements(drivetrainSub);// Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Use open-loop control for drive motors
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_apriltagPos = m_visionSub.getTagPose2d();
    //check if angle is positive or negative

    System.out.println(m_visionSub.getTx());

    // if(m_visionSub.getTx() > 5 || m_visionSub.getTx() < -5) {
    //   System.out.println("aa");
    //   if(m_visionSub.getTx() > 0) {
    //     System.out.println("bb");
    //     m_drivetrainSub.setControl(autoDrive.withVelocityX(0).withVelocityY(0)
    //         .withRotationalRate(-MaxAngularRate / 10));//RotationsPerSecond.of(-0.1).in(RadiansPerSecond)));
    //     // m_drivetrainSub 
    //     //     .applyRequest(() -> autoDrive.withVelocityX(0).withVelocityY(0)
    //     //         .withRotationalRate(RotationsPerSecond.of(-0.75).in(RadiansPerSecond)));
    //   } else {
    //     System.out.println("cc");
    // m_drivetrainSub.setControl(autoDrive.withVelocityX(0).withVelocityY(0)
    //.withRotationalRate(MaxAngularRate / 10));//RotationsPerSecond.of(0.1).in(RadiansPerSecond)));
    // m_drivetrainSub
    //     .applyRequest(() -> autoDrive.withVelocityX(0).withVelocityY(0)
    //         .withRotationalRate(RotationsPerSecond.of(0.75).in(RadiansPerSecond)));
    //}
    //} else {
    //xSystem.out.println("dd");
    xDist = m_apriltagPos.getX();
    yDist = m_apriltagPos.getY();
    double totalDist = Math.sqrt((xDist * xDist) + (yDist * yDist));
    double xPower = xDist / totalDist;
    double yPower = yDist / totalDist;
    m_drivetrainSub.setControl(
        autoDrive.withVelocityX(-yPower * MaxSpeed / 10).withVelocityY(xPower * MaxSpeed / 10)
            .withRotationalRate(-m_visionSub.getRobotRotation() / ((xDist * 50) + 50) * MaxAngularRate * 0.10));//applyRequest(() -> autoDrive.withVelocityX(xDist).withVelocityY(yDist));

    //}

    System.out.println(xDist);
    System.out.println(yDist);

    if(m_visionSub.getTv() == 0) {
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
    System.out.println("end");
    m_drivetrainSub.setControl(autoDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    m_drivetrainSub.setControl(brake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println(yDist);
    //System.out.println(counter);
    //xDist < 0.05 && xDist > -0.05 && yDist < 0.45 && yDist > .3
    if(Math.abs(yDist) < 0.3 && m_visionSub.getTv() != 0) {
      System.out.println("yDist" + yDist);

      return true;
    }
    if(counter >= 25) {
      System.out.println("counter: " + counter);
      return true;
    }
    return false;
  }
}
