// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ElevatorSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ElevatorMoveWithJoystickCmd extends Command {
  private final CommandPS4Controller m_controller;
  private final ElevatorSub m_elevatorSub;
  private boolean m_wasInDeadZone = true;

  /** Creates a new ElevatorWithJoystickCmd. */
  public ElevatorMoveWithJoystickCmd(CommandPS4Controller controller, ElevatorSub elevatorSub) {
    m_controller = controller;
    m_elevatorSub = elevatorSub;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("*********ElevatorWithJoystickCmd Run*********"); // Debug only
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("ElevatorWithJoystickCmd " + m_elevatorSub.getPosition()); // Debug only
    // get controller joystick value
    double elevatorPower = -m_controller.getLeftY();

    // create deadband if power is less than 5%
    if(Math.abs(elevatorPower) < 0.05) {
      m_wasInDeadZone = true;
    } else {
      m_wasInDeadZone = false;
    }
    // Turns off automatic height control to allow joystick use.
    if(!m_wasInDeadZone) {
      m_elevatorSub.runHeightControl(false); // TODO: Consider using a flag to turn this on/off and letting the subsystem's periodic() call this method
      m_elevatorSub.setPower(elevatorPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
