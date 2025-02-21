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
public class L4PlacementGrp extends SequentialCommandGroup {
  /** Creates a new L4PlacementGrp. */
  public L4PlacementGrp(ArmSub armSub, ElevatorSub elevatorSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> elevatorSub.setTargetHeight(525)), // just high enough for arm with coral to be able to go to 165
        new WaitCommand(2),
        new InstantCommand(() -> armSub.setTargetAngle(165)), // 165 is just high enough to get the coral to a height that it can get to the branch
        new WaitCommand(5),
        new InstantCommand(() -> elevatorSub.setTargetHeight(1070)), // as tall as possible so that the coral can reach l4
        new WaitCommand(5), // wait a second for the driver or a vision command to align the robot with a branch
        new InstantCommand(() -> armSub.setTargetAngle(135)), // down enough to put the coral on the branch
        new WaitCommand(5),
        new InstantCommand(() -> elevatorSub.setTargetHeight(650)), // Lower elevator to remove coral
        new WaitCommand(5), // Wait time for the driver to move back so that the arm does not hit any other branches
        new InstantCommand(() -> elevatorSub.setTargetHeight(440)), //use the resting elevator height, this is an estimation
        new WaitCommand(2),
        new InstantCommand(() -> armSub.setTargetAngle(0)) // set the arm back to the starting angle
    );
  }

}
