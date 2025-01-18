// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import java.time.Duration;
import java.time.Instant;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSub;
import frc.robot.utils.TestManager;
import frc.robot.utils.TestManager.*;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class TestIntakeSub extends Command {
  private final IntakeSub m_intakeSub;
  private final TestManager m_testManager;
  private final int m_testId;
  private Instant m_startTime;
  private boolean m_abortTest = false;

  /** Creates a new TestIntakeSub. */
  public TestIntakeSub(IntakeSub intakeSub, TestManager testManager) {
    m_intakeSub = intakeSub;
    m_testManager = testManager;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);

    m_testId = m_testManager.registerNewTest("Intake 1");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset the encoder and run the motor for a given time
    m_startTime = Instant.now();
    //m_intakeSub.resetPosition();
    //StatusCode sc = m_intakeSub.runMotor(Constants.Tests.kDriveMotorPower);
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
      m_intakeSub.startIntake(0.0);
      m_testManager.updateTestStatus(m_testId, Result.kFail, "Test interrupted");
      return;
    }

    // Before turning off the motor, read the current current (amps) and position
    double currentAmps = m_intakeSub.getAmps();
    double currentPosition = m_intakeSub.getPosition();

    // Stop the motor
    m_intakeSub.startIntake(0.0);

    // Check to see if the measured current is good, ok or bad
    TestManager.Result ampResult = m_testManager.determineResult(currentAmps, Constants.Tests.kIntakeMotorExpectedAmps,
        Constants.Tests.kIntakeMotorAmpsTolerance, Constants.Tests.kIntakeMotorAmpsMinimum);
    String ampsText = "Amps=" + currentAmps + " (Target=" + Constants.Tests.kIntakeMotorExpectedAmps + "+/-"
        + Constants.Tests.kIntakeMotorAmpsTolerance + ")";
    System.out.println("IntakebSub " + ampsText);

    // Check to see if the measured position is good, ok or bad
    TestManager.Result positionResult =
        m_testManager.determineResult(currentPosition, Constants.Tests.kIntakeMotorExpectedPosition,
            Constants.Tests.kIntakeMotorPositionTolerance, Constants.Tests.kIntakeMotorPositionMinimum);
    String positionText =
        "Position=" + currentPosition + " (Target=" + Constants.Tests.kIntakeMotorExpectedPosition + "+/-"
            + Constants.Tests.kIntakeMotorPositionTolerance + ")";
    System.out.println("IntakeSub " + positionText);


    // Figure out the overall test result
    TestManager.Result testResult = TestManager.Result.kPass;
    if((ampResult == TestManager.Result.kFail) || (positionResult == TestManager.Result.kFail)) {
      testResult = TestManager.Result.kFail;
    } else if((ampResult == TestManager.Result.kWarn) || (positionResult == TestManager.Result.kWarn)) {
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
