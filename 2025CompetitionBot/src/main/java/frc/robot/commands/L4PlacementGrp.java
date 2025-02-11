// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ElevatorSub;
import edu.wpi.first.wpilibj.Watchdog;
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
  public L4PlacementGrp(ArmSub armSub, ElevatorSub elevatorSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> elevatorSub.setTargetHeight(1000)), // as tall as possible so that the bottom of the coral is fully above the top of l4
        new WaitCommand(1), // wait a second for the instant command
        new SetArmToPositionCmd(135, armSub), // 90 degrees because the coral will just slam straight down onto l4
        new WaitCommand(1), // wait a second for the instant command
        new InstantCommand(() -> elevatorSub.setTargetHeight(440)), //use the resting elevator height, this is an estimation
        new WaitCommand(0.5), // wait half a second for the instant command
        new SetArmToPositionCmd(0, armSub) // set the arm back to the 
    );
  }

}
