// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ElevatorSub;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L4PlacementGrp extends SequentialCommandGroup {
  /** Creates a new L4PlacementGrp. */
  public L4PlacementGrp(CommandPS4Controller driverController, ArmSub armSub, ElevatorSub elevatorSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetElevatorToHeightCmd(1900, elevatorSub), // as tall as possible so that the bottom of the coral is fully above the top of l4
        new SetArmToPositionCmd(90, armSub), // 90 degrees because the coral will just slam straight down onto l4
        new SetElevatorToHeightCmd(440, elevatorSub) //use the resting elevator height, this is an estimation
    );
  }

}
