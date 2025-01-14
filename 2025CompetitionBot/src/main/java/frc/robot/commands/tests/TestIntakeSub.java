// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import java.time.Instant;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSub;
import frc.robot.utils.TestManager;

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
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
