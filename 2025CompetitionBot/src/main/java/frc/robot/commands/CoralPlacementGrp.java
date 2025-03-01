// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralPlacementGrp extends SequentialCommandGroup {
  double m_scoringBranch;

  private enum CommandSelector {
    ZERO, TWO, THREE, FOUR
  }

  private CommandSelector select() {
    if(m_scoringBranch == 4) {
      return CommandSelector.FOUR;
    } else {
      if(m_scoringBranch == 3) {
        return CommandSelector.THREE;
      } else {
        if(m_scoringBranch == 2) {
          return CommandSelector.TWO;
        } else {
          return CommandSelector.ZERO;
        }
      }
    }
  }

  /** Creates a new CoralPlacementGrp. */
  public CoralPlacementGrp(ElevatorSub elevatorSub, ArmSub armSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    new SelectCommand<>(
        // Maps selector values to commands
        Map.ofEntries(
            Map.entry(CommandSelector.FOUR, new SetElevatorToHeightCmd(750, elevatorSub)),
            Map.entry(CommandSelector.FOUR, new WaitCommand(1)), // in seconds
            Map.entry(CommandSelector.FOUR, new SetArmToPositionCmd(-90, armSub)), // most of these values could be changed 
            Map.entry(CommandSelector.FOUR,
                new SetElevatorToHeightCmd(Constants.Elevator.kCoralGrabableHeight, elevatorSub)),
            Map.entry(CommandSelector.THREE, new SetElevatorToHeightCmd(630, elevatorSub)), // PLEASE CHANGE THIS VALUE
            Map.entry(CommandSelector.THREE, new SetArmToPositionCmd(-90, armSub)),
            Map.entry(CommandSelector.THREE,
                new SetElevatorToHeightCmd(Constants.Elevator.kCoralGrabableHeight, elevatorSub)),
            Map.entry(CommandSelector.TWO, new SetElevatorToHeightCmd(630, elevatorSub)), // PLEASE CHANGE THIS VALUE
            Map.entry(CommandSelector.TWO, new SetArmToPositionCmd(-90, armSub)),
            Map.entry(CommandSelector.TWO,
                new SetElevatorToHeightCmd(Constants.Elevator.kCoralGrabableHeight, elevatorSub)),
            Map.entry(CommandSelector.ZERO, new PrintCommand("Stop pressing this when it doesn't have a value"))),
        this::select);
  }
}
