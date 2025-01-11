// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.KrakenTestSub;
import frc.robot.subsystems.NeoTestSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.comands.DriveCmd;
import frc.robot.comands.PivotToAngleCmd;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final KrakenTestSub m_krakenSub = new KrakenTestSub();
  private final NeoTestSub m_neoTestSub = new NeoTestSub();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //m_krakenSub.setDefaultCommand(new RunCommand(() -> m_krakenSub.drive(-m_driverController.getLeftY()), m_krakenSub));
    // m_neoTestSub
    //     .setDefaultCommand(new RunCommand(() -> m_neoTestSub.drive(m_driverController.getRightY()), m_neoTestSub));

    m_neoTestSub.setDefaultCommand(
        new DriveCmd(m_driverController, m_neoTestSub, m_krakenSub));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.square().onTrue(new PivotToAngleCmd(45, m_neoTestSub));

    m_driverController.share().onTrue(new InstantCommand(() -> m_neoTestSub.resetEncoder()));
    //       .onTrue();

    // m_operatorController.square()
    // .onTrue(new ShooterPrepGrp(Constants.Shooter.kAngleAutoLine, Constants.Flywheel.kFlywheelShootVelocity,
    //     m_arduinoSub, m_feederSub, m_flywheelSub,
    //     m_pivotSub, m_ledSub));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("No Autos");
  }
}
