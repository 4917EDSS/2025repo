// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.CanSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.VisionSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCoralScoreL4Grp extends SequentialCommandGroup {
  /** Creates a new AutoCoralScoreL4Grp. */
  public AutoCoralScoreL4Grp(ArmSub armSub, CanSub canSub, DrivetrainSub drivetrainSub,
      ElevatorSub elevatorSub, VisionSub visionSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new MoveElArmGrp(Constants.Elevator.kL4PreScoreHeight, Constants.Arm.kL4PreScoreAngle, armSub, elevatorSub), //Move to pre score position
            new AutoDriveCmd(visionSub, drivetrainSub, true) //Drive to score location
        ),
        new MoveElArmGrp(Constants.Elevator.kL4PostScoreHeight, Constants.Arm.kL4PostScoreAngle, armSub, elevatorSub), //Move to post score location (score)
        new BackUpAfterScoringCmd(drivetrainSub) //Back up
    //new ScheduleCommand(new AutoGrabCoralGrp(armSub, canSub, elevatorSub)) //Grab coral

    );
  }
}
