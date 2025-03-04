// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
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
        new ParallelCommandGroup(
            new SetElevatorToHeightCmd(Constants.Elevator.kL4PreScoreHeight, elevatorSub) //its set to max height and itll stop once the coral gets to the height where it can hit part of the elevator so then the arm will move out and it will resume going up
        //             new SetArmToPositionCmd(Constants.Arm.kMaxArmAngle, armSub)),
        //         new WaitCommand(2.0),
        //         new ParallelCommandGroup(
        // //            new SetArmToPositionCmd(0.0, armSub),
        //             new SetElevatorToHeightCmd(Constants.Elevator.kL4PostScoreHeight - 300, elevatorSub))
        ));
  }

}


//1331 MM
//32 deg
