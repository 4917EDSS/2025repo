// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ElevatorSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCoralScoreL2ArmElGrp extends SequentialCommandGroup {

  /** Creates a new AutoCoralScoreL2Grp. */
  public AutoCoralScoreL2ArmElGrp(ArmSub armSub, ElevatorSub elevatorSub) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveElArmGrp(Constants.Elevator.kCoralGrabbableHeight, Constants.Arm.kL2PreScoreAngle, armSub, elevatorSub), //Move to pre score position
      new MoveElArmGrp(Constants.Elevator.kL2PreScoreHeight, Constants.Arm.kL2PreScoreAngle, armSub, elevatorSub)
    );
  }
}
