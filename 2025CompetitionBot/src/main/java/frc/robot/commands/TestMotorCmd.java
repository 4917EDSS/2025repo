// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.KrakenSub;
import frc.robot.subsystems.MotorSub;
import frc.robot.utils.TestManager;
import frc.robot.utils.TestManager.Result;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class TestMotorCmd extends Command {
  private MotorSub m_subsystem;
  private final TestManager m_testManager;
  private int m_testId;
  private Instant m_startTime;
  private boolean m_abortTest = false;

  /** Creates a new TestElevatorCmd. */
  // public TestElevatorCmd(KrakenSub krakenSub, TestManager testManager) {
  //   m_krakenSub = krakenSub;
  //   m_testManager = testManager;

  //   // Use addRequirements() here to declare subsystem dependencies.
  //   addRequirements(krakenSub);

  //   m_testId = m_testManager.registerNewTest("ElevatorKrakenLeftMotor");
  //   TestKrakenAll(krakenSub, testManager, m_testId);
  // }

  public TestMotorCmd(TestManager testManager, MotorSub subsystem, int testId) {
    m_testManager = testManager;
    m_subsystem = subsystem;
    m_testId = testId;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Instant.now();
    m_subsystem.resetPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_abortTest) {
      // Already stopped the motor and reported the test results, just quit
      return;
    }

    if(interrupted) {
      m_subsystem.runMotor(0.0);
      m_testManager.updateTestStatus(m_testId, Result.kFail, "Test interrupted");
      return;
    }

    double currentAmps = m_subsystem.getAmps();
    double currentPosition = m_subsystem.getPosition();

    // Stop the motor
    m_subsystem.runMotor(0.0);

    // Check to see if the measured current is good, ok or bad
    TestManager.Result ampsResult = m_testManager.determineResult(currentAmps, Constants.Tests.kDriveMotorExpectedAmps,
        Constants.Tests.kDriveMotorAmpsTolerance, Constants.Tests.kDriveMotorAmpsMinimum);
    String ampsText = "Amps=" + currentAmps + " (Target=" + Constants.Tests.kDriveMotorExpectedAmps + "+/-"
        + Constants.Tests.kDriveMotorAmpsTolerance + ")";
    System.out.println("KrakenSub " + ampsText);

    // Check to see if the measured position is good, ok or bad
    TestManager.Result positionResult =
        m_testManager.determineResult(currentPosition, Constants.Tests.kDriveMotorExpectedPosition,
            Constants.Tests.kDriveMotorPositionTolerance, Constants.Tests.kDriveMotorPositionMinimum);
    String positionText =
        "Position=" + currentPosition + " (Target=" + Constants.Tests.kDriveMotorExpectedPosition + "+/-"
            + Constants.Tests.kDriveMotorPositionTolerance + ")";
    System.out.println("KrakenSub " + positionText);

    // Figure out the overall test result
    TestManager.Result testResult = TestManager.Result.kPass;
    if((ampsResult == TestManager.Result.kFail) || (positionResult == TestManager.Result.kFail)) {
      testResult = TestManager.Result.kFail;
    } else if((ampsResult == TestManager.Result.kWarn) || (positionResult == TestManager.Result.kWarn)) {
      testResult = TestManager.Result.kWarn;
    }

    // Update the test results
    m_testManager.updateTestStatus(m_testId, testResult, ampsText + " " + positionText);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Check if we should abort
    if(m_abortTest) {
      return true;
    }
    // Otherwise, wait for the test time to have elapsed
    else if(Duration.between(m_startTime, Instant.now()).toMillis() > Constants.Tests.kDriveMotorTimeMs) {
      return true;
    }
    return false;
  }
}
