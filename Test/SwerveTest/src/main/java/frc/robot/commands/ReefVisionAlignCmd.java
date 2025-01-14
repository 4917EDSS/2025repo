// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSub;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Transform2d;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ReefVisionAlignCmd extends Command {
  private final VisionSub m_visionSub;
  private static Rotation2d targetAlignment;
  private final CommandSwerveDrivetrain drivetrain;
  SwerveRequest.PointWheelsAt driveToPos;
  private double offset;

  /** Creates a new ReefVisionAlignCmd. */
  public ReefVisionAlignCmd(VisionSub visionSub, double offset) {
    this.offset = offset;
    drivetrain = RobotContainer.drivetrain;
    m_visionSub = visionSub;
    addRequirements(visionSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("init");
    driveToPos =
        new SwerveRequest.PointWheelsAt().withModuleDirection(new Rotation2d(0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.applyRequest(() -> driveToPos.withModuleDirection(m_visionSub
        .getTargetAngle(m_visionSub.getTarget2D().transformBy(new Transform2d(offset, 0, new Rotation2d(0))))));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_visionSub.getTarget2D().getX() < 0.1 && m_visionSub.getTarget2D().getX() > -0.1
        && m_visionSub.getTarget2D().getY() < 0.1 && m_visionSub.getTarget2D().getY() > -0.1) {
      System.out.println(m_visionSub.simpleHasTarget());
      return true;
    } else {
      return false;
    }
  }
}
