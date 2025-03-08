// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.CanSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.utils.RobotState;


// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAlgaeRemovalL3L4Grp extends SequentialCommandGroup {
  /** Creates a new AlgaeRemovalL3L4Grp. */
  public AutoAlgaeRemovalL3L4Grp(ArmSub armSub, CanSub canSub, DrivetrainSub drivetrainSub,
      ElevatorSub elevatorSub,
      VisionSub visionSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new MoveElArmGrp(Constants.Elevator.kL3L4AlgaeRemovalPrepHeight, Constants.Arm.kL3L4AlgaeRemovalPrepAngle,
            armSub, elevatorSub), // Move elevator and arm to algae removal location
        new AutoDriveCmd(visionSub, drivetrainSub, false), // Drive to vision target
        new MoveElArmGrp(Constants.Elevator.kL3L4AlgaeRemovalPostHeight, Constants.Arm.kL3L4AlgaeRemovalPostAngle,
            armSub, elevatorSub), // Remove algae
        new BackUpAfterScoringCmd(drivetrainSub), // Backup
        new ScheduleCommand(new AutoGrabCoralGrp(armSub, canSub, elevatorSub)), //Pick up coral
        new InstantCommand(() -> RobotState.l3L4Algae()));

  }
}
