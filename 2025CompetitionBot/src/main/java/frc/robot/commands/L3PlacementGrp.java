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
public class L3PlacementGrp extends SequentialCommandGroup {
  /** Creates a new L3PlacementGrp. */
  public L3PlacementGrp(ArmSub armSub, ElevatorSub elevatorSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetElevatorToHeightCmd(600, elevatorSub),
        new SetArmToPositionCmd(32, armSub),
        new SetElevatorToHeightCmd(764, elevatorSub)
    // new InstantCommand(() -> elevatorSub.setTargetHeight(764)), // Set height close to L3
    // new WaitCommand(2), // Wait 2 seconds
    // new InstantCommand(() -> armSub.setTargetAngle(31)), // Move arm angle above L3
    // new WaitCommand(2)
    //new WaitCommand(5), // Wait 5 second
    //new InstantCommand(() -> elevatorSub.setTargetHeight(500)), // Bring elevator height down to drop coral
    //new WaitCommand(2), // Wait 2 seconds
    //new InstantCommand(() -> elevatorSub.setTargetHeight(212)) // Bring elevator height down
    );

    //new SetElevatorToHeightCmd(400, elevatorSub)); // Bring elevator height down
  }

}


//764mm
//31 deg
