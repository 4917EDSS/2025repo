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
public class L3PlacementGrp extends SequentialCommandGroup {
  /** Creates a new L3PlacementGrp. */
  public L3PlacementGrp(ArmSub armSub, ElevatorSub elevatorSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> elevatorSub.setTargetHeight(700)), // Set height close to L3
        new WaitCommand(2), // Wait 2 seconds
        new SetArmToPositionCmd(160, armSub), // Move arm angle above L3
        new SetArmToPositionCmd(130, armSub), // Bring arm down to place coral onto L3
        new InstantCommand(() -> elevatorSub.setTargetHeight(400))); // Bring elevator height down
  }
}
