// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ElevatorSub;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L2PlacementGrp extends SequentialCommandGroup {
  /** Creates a new L2PlacementGrp. */
  public L2PlacementGrp(ArmSub armSub, ElevatorSub elevatorSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> elevatorSub.setTargetHeight(487)), // Set height to clear frame when moving arm
        new WaitCommand(2),
        new InstantCommand(() -> armSub.setTargetAngle(159)), // Move arm angle above L2
        new WaitCommand(2),
        new InstantCommand(() -> elevatorSub.setTargetHeight(54)),
        new WaitCommand(5),
        new InstantCommand(() -> armSub.setTargetAngle(139)), // Set height to near L2
        new WaitCommand(2),
        new InstantCommand(() -> elevatorSub.setTargetHeight(5))); // Move elevator down to place coral
  }
}
