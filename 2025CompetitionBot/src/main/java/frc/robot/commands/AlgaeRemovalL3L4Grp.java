// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ElevatorSub;


// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeRemovalL3L4Grp extends SequentialCommandGroup {
  /** Creates a new AlgaeRemovalL3L4Grp. */
  public AlgaeRemovalL3L4Grp(ArmSub armSub, ElevatorSub elevatorSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> armSub.setTargetAngle(83.2), armSub),
        new WaitCommand(2), // Wait 2 seconds
        new InstantCommand(() -> elevatorSub.setTargetHeight(700), elevatorSub), // Set elevator below 
        new WaitCommand(2),
        new InstantCommand(() -> armSub.setTargetAngle(100), armSub),
        new WaitCommand(2));
  }
}
