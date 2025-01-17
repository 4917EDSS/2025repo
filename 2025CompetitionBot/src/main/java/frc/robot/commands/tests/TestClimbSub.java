// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import java.time.Duration;
import java.time.Instant;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSub;
import frc.robot.utils.TestManager;
import frc.robot.utils.TestManager.Result;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class TestClimbSub extends Command {
  private final ClimbSub m_climbSub;
  private final TestManager m_testManager;
  private final int m_testId;
  private Instant m_startTime;
  private boolean m_abortTest = false;

  /** Creates a new TestClimbSub. */
  public TestClimbSub(ClimbSub climbSub, TestManager testManager) {
    m_climbSub = climbSub;
    m_testManager = testManager;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSub);

    m_testId = m_testManager.registerNewTest("Climb Motor 1");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset the encoder and run the motor for a given time
    m_startTime = Instant.now();
    m_climbSub.resetPosition();
    m_climbSub.moveClimb(Constants.Tests.kDriveMotorPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Nothing to do here but wait for the test to be over
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_abortTest) {
      // Already stopped the motor and reported the test results, just quit
      return;
    }

    if(interrupted) {
      m_climbSub.moveClimb(0.0);
      m_testManager.updateTestStatus(m_testId, Result.kFail, "Test interrupted");
      return;
    }

    // Before turning off the motor, read the current current (amps) and position
    double currentAmps = m_climbSub.getAmps();
    double currentPosition = m_climbSub.getDistance();

    // Stop the motor
    m_climbSub.moveClimb(0.0);

    // Check to see if the measured current is good, ok or bad
    TestManager.Result ampResult = m_testManager.determineResult(currentAmps, Constants.Tests.kDriveMotorExpectedAmps,
        Constants.Tests.kDriveMotorAmpsTolerance, Constants.Tests.kDriveMotorAmpsMinimum);
    String ampsText = "Amps=" + currentAmps + " (Target=" + Constants.Tests.kDriveMotorExpectedAmps + "+/-"
        + Constants.Tests.kDriveMotorAmpsTolerance + ")";
    System.out.println("ClimbSub " + ampsText);

    // Check to see if the measured position is good, ok or bad
    TestManager.Result positionResult =
        m_testManager.determineResult(currentPosition, Constants.Tests.kDriveMotorExpectedPosition,
            Constants.Tests.kDriveMotorPositionTolerance, Constants.Tests.kDriveMotorPositionMinimum);
    String positionText =
        "Position=" + currentPosition + " (Target=" + Constants.Tests.kDriveMotorExpectedPosition + "+/-"
            + Constants.Tests.kDriveMotorPositionTolerance + ")";
    System.out.println("ClimbSub " + positionText);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
