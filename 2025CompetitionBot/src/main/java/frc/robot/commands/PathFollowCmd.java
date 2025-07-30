// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.PathGenCmd;
import frc.robot.subsystems.DrivetrainSub;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicIntegerArray;
import java.util.concurrent.atomic.AtomicLong;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class PathFollowCmd extends Command {

  private final DrivetrainSub m_drivetrainSub;
  final PathGenCmd m_pathGenCmd = new PathGenCmd();
  ArrayList<int[]> path = new ArrayList<int[]>();
  AtomicIntegerArray currentPos;
  int conversionFactor;
  int fieldLength = 57; //I actually have no idea, were gonna have to figure this one out
  int[] targetPos;

  /** Creates a new PathFollowCmd. */
  public PathFollowCmd(DrivetrainSub drivetrainSub, int[] target) {
    targetPos = target;
    conversionFactor = fieldLength / m_pathGenCmd.field.length;
    m_drivetrainSub = drivetrainSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double xPosDiff = m_drivetrainSub.getPose().getX() * conversionFactor - currentPos.get(0);
    double yPosDiff = m_drivetrainSub.getPose().getX() * conversionFactor - currentPos.get(1);
    //path = m_pathGenCmd.generatePath(currentPos, targetPos, m_pathGenCmd.field);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //path = m_pathGenCmd.generatePath(null, null, null);
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
