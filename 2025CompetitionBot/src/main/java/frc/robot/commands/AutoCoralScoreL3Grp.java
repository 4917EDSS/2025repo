// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCoralScoreL3Grp extends SequentialCommandGroup {
  /** Creates a new AutoCoralScoreL3Grp. */
  public AutoCoralScoreL3Grp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    // TODO:  use bulding-block command, MoveElArmGrp and MoveRelElAbsArmGrp, for all elevator and arm movements
    // Move elevator and arm to pre score location
    // Drive to vision target
    // Score
    // Backup
    // Setup to grab next coral
    );
  }
}
