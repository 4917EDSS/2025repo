// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.comands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.KrakenTestSub;
import frc.robot.subsystems.NeoTestSub;
import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


// Called when the command is initially scheduled.

public class MoveNEOMotorCmd extends InstantCommand {
  private static Logger m_logger = Logger.getLogger(MoveNEOMotorCmd.class.getName());

  private final CommandPS4Controller m_controller;
  private final NeoTestSub m_neotestSub;
  private final KrakenTestSub m_krakenTestSub;


  public MoveNEOMotorCmd(CommandPS4Controller controller, NeoTestSub neotestSub, KrakenTestSub krakentestSub) {
    m_controller = controller;
    m_neotestSub = neotestSub;
    m_krakenTestSub = krakentestSub;

    addRequirements(neotestSub);
  }

  @Override
  public void execute() {
    // get controller joystick value
    double pivotPowerLeft = -m_controller.getLeftY(); // NEO motor
    double pivotPowerRight = -m_controller.getRightY(); // Kraken motor
    //Deadband
    m_logger.info("pivot power" + pivotPowerLeft);
    if(Math.abs(pivotPowerLeft) > 0.05) {
      m_neotestSub.setPower(pivotPowerLeft); // (pivotPowerLeft);
    } else {
      m_neotestSub.setPower(0.0);
    }

    //Deadband
    m_logger.info("pivot power" + pivotPowerRight);
    if(Math.abs(pivotPowerRight) > 0.05) {
      m_krakenTestSub.drive(pivotPowerRight);
    } else {
      m_krakenTestSub.drive(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

