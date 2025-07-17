// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Commands.PathGenCmd;
import frc.robot.utils.FieldImage;

public class RobotContainer {
  private final CommandPS4Controller m_controller = new CommandPS4Controller(0);
  private final PathGenCmd m_pathGenCmd = new PathGenCmd();
  private final FieldImage m_fieldImage = new FieldImage();
  private int[] startingPos = {0, 0};
  private int[] targetPos = {20, 10};

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_controller.square()
        .onTrue(new PrintCommand(m_pathGenCmd.printPoints(startingPos, targetPos, m_fieldImage.field)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
