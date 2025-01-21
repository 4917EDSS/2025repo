// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class SetElevatorToHeightCmd extends Command {
  private final Double m_targetHeight;
  private final ElevatorSub m_elevatorSub;

  /** Creates a new SetElevatorToHeightCmd. */
  public SetElevatorToHeightCmd(double height, ElevatorSub elevatorSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_targetHeight = height;
    m_elevatorSub = elevatorSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSub.setElevatorMotor(0.25);
    System.out.println("*********SetElevatorHeightCmd Run*********");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSub.setElevatorMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //  if(m_elevatorSub.getHeight().gte(m_targetHeight)) {
    if(m_elevatorSub.getHeight() > (m_targetHeight)) {
      return true;
    }
    return false;
  }
}
