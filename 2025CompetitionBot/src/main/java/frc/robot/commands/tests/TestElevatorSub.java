// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.Command;
import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.utils.TestManager;
import frc.robot.utils.TestManager.Result;


/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class TestElevatorSub extends Command {
  private final ElevatorSub m_elevatorSub;
  private final TestManager m_testManager;
  private final int[] m_testIds;
  private Instant m_startTime;

  /** Creates a new TestElevatorSub. */
  public TestElevatorSub(ElevatorSub elevatorSub, TestManager testManager) {
    m_elevatorSub = elevatorSub;
    m_testManager = testManager;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(elevatorSub);
    m_testIds = new int[2];
    m_testIds[0] = m_testManager.registerNewTest("Elevator Motor 1");
    m_testIds[1] = m_testManager.registerNewTest("Elevator Motor 2");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Instant.now();
    //  m_elevatorSub.testElevatorMotor(Constants.Tests.kElevatorMotor);
    // m_elevatorSub.testElevatorMotor(Constants.Tests.kElevatorMotor2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      m_elevatorSub.testElevatorMotor(0);
      m_testManager.updateTestStatus(m_testIds[0], Result.kFail, "Test Interrupted");
      m_testManager.updateTestStatus(m_testIds[1], Result.kFail, "Test Interrupted");
      return;
    }
    double[] currentPositions = {
        // m_elevatorSub.getHeight(),
    };

    for(int motorId = 0; motorId < m_testIds.length; motorId++) {
      // TestManager.Result positionResult = 
      // m_testManager.determineResult(m_testIds[motorId], Constants.Tests.kElevator)
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
