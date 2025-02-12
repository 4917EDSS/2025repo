// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.TestManager;
import frc.robot.utils.TestMotorParameters;
import frc.robot.utils.TestableSubsystem;
import frc.robot.utils.TestManager.Test;
import frc.robot.utils.TestManager.TestResult;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class TestMotorCmdAltOne extends Command {
  private final TestMotorParameters m_parameters;
  private final TestableSubsystem m_subsystem;
  //private final int m_testId;
  private Test m_test;

  private Instant m_startTime;


  /** Creates a new TestMotorCmd. */
  public TestMotorCmdAltOne(TestMotorParameters parameters, TestableSubsystem subsystem, TestManager testManager) {
    m_parameters = parameters;
    m_subsystem = subsystem;
    m_test = testManager.getTest(m_parameters.kTestName);
    //m_testId = testManager.registerNewTest(m_parameters.kTestName);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.testEnableMotorTestMode(m_parameters.kMotorId); //no code for this yet
    m_startTime = Instant.now();

    m_subsystem.testSetMotorPower(m_parameters.kMotorId, 0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      // Test was interrupted.  Stop the motor and fail the test
      m_subsystem.testSetMotorPower(m_parameters.kMotorId, 0.0);
      m_test.interrupt();
      return;
    }

    double currentAmps = m_subsystem.testGetMotorAmps(m_parameters.kMotorId);
    double currentPosition = m_subsystem.testGetMotorPosition(m_parameters.kMotorId);

    // Test done.  Stop the motor
    m_subsystem.testSetMotorPower(m_parameters.kMotorId, 0.0);


 // Check to see if the measured current is good, ok or bad
    TestResult ampsResult = m_test.addResult("Amps")
      .withActualValue(currentAmps)
      .withTargetValue(m_parameters.kAmpsTarget)
      .withFailTolerance(m_parameters.kAmpsTolerance)
      .withWarnTolerance(m_parameters.kAmpsMin);
    // it would be nice to be able to print result out
    System.out.println(ampsResult);

    // Check to see if the measured position is good, ok or bad
    TestResult positionResult = m_test.addResult("Position")
      .withActualValue(currentPosition)
      .withTargetValue(m_parameters.kPositionTarget)
      .withFailTolerance(m_parameters.kPositionTolerance)
      .withWarnTolerance(m_parameters.kPositionMin);
    // it would be nice to print the result out
    System.out.println(positionResult);

    // update the results
    m_test.updateStatus();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Check to see if the test time has elapsed
    if(Duration.between(m_startTime, Instant.now()).toMillis() >= m_parameters.kTimeMs) {
      return true;
    }
    return false;
  }
}
