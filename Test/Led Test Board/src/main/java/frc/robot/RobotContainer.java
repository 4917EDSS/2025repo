// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.LedSub;

public class RobotContainer {
  private final LedSub m_ledSub = new LedSub();
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }


  private void configureBindings() {
    // m_driverController.square().onTrue(new LedCmdGrp(m_ledSub));
    m_driverController.cross().onTrue(new InstantCommand(() -> m_ledSub.turnOnHeadlights(), m_ledSub))
        .onFalse(new InstantCommand(() -> m_ledSub.turnOffHeadlights(), m_ledSub));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
