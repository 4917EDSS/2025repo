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
public class L2PlacementGrp extends SequentialCommandGroup {
  /** Creates a new L2PlacementGrp. */
  public L2PlacementGrp(ArmSub armSub, ElevatorSub elevatorSub, CoralPlacementGrp coralPlacementGrp,
      double scoringBranch) {
    double m_scoringBranch = scoringBranch;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetElevatorToHeightCmd(352, elevatorSub),
        new SetArmToPositionCmd(32, armSub)
    // new InstantCommand(() -> elevatorSub.setTargetHeight(352)), // Set height to clear frame when moving arm
    // new WaitCommand(2),
    // new InstantCommand(() -> armSub.setTargetAngle(32)), // Move arm angle above L2
    // new WaitCommand(2)
    // new WaitCommand(2),
    // new InstantCommand(() -> elevatorSub.setTargetHeight(54)),
    // new WaitCommand(5),
    // new InstantCommand(() -> armSub.setTargetAngle(49)), // Set height to near L2
    // new WaitCommand(2),
    // new InstantCommand(() -> elevatorSub.setTargetHeight(5))); // Move elevator down to place coral
    );
    scoringBranch = 2;
  }
}


//352 mm
//32 deg
