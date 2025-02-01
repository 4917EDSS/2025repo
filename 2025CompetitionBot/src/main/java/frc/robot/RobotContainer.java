// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import java.util.*;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DoNothingGrp;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmMoveWithJoystickCmd;
import frc.robot.commands.AutoDriveCmd;
import frc.robot.commands.ElevatorMoveWithJoystickCmd;
import frc.robot.commands.KillAllCmd;
import frc.robot.commands.SetArmToPositionCmd;
import frc.robot.commands.SetElevatorToHeightCmd;
import frc.robot.commands.tests.RunTestsGrp;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.utils.SwerveTelemetry;
import frc.robot.utils.TestManager;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  boolean isLimelight = true;
  // Swerve constants and objects (from CTRE Phoenix Tuner X)
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveTelemetry swerveLogger = new SwerveTelemetry(MaxSpeed);

  // RobotContainer constants
  private final TestManager m_testManager = new TestManager();

  // Robot subsystems
  //private final ArduinoSub m_arduinoSub = new ArduinoSub();   // TODO:  Implement new CAN Arduino
  private final ClimbSub m_climbSub = new ClimbSub();
  public final DrivetrainSub m_drivetrainSub = TunerConstants.createDrivetrain();
  private final ElevatorSub m_elevatorSub = new ElevatorSub();
  private final IntakeSub m_intakeSub = new IntakeSub();
  //private final LedSub m_ledSub = new LedSub(m_arduinoSub);  // TODO:  Implement with new Arduino
  private final VisionSub m_visionSub;
  private final ArmSub m_armSub = new ArmSub();

  // Controllers
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final CommandPS4Controller m_operatorController =
      new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);

  // RobotContainer member variables
  public static boolean disableShuffleboardPrint = true;

  SendableChooser<Command> m_Chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NetworkTableEvent.Kind[] topicsArray = NetworkTableEvent.Kind.values();
    if(!Arrays.asList(topicsArray).contains("limelight")) {
      isLimelight = false;
      m_visionSub = null;
    } else {
      m_visionSub = new VisionSub(m_drivetrainSub);
    }

    m_testManager.setTestCommand(new RunTestsGrp(m_climbSub, m_intakeSub, m_testManager));

    // Default commands
    m_drivetrainSub.setDefaultCommand(

        // Note: X is defined as forward and Y as left according to WPILib convention
        m_drivetrainSub.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    m_armSub.setDefaultCommand(new ArmMoveWithJoystickCmd(m_operatorController, m_armSub));
    m_elevatorSub.setDefaultCommand(new ElevatorMoveWithJoystickCmd(m_operatorController, m_elevatorSub));

    // Register Swerve telemetry
    m_drivetrainSub.registerTelemetry(swerveLogger::telemeterize);

    // Configure the trigger bindings
    configureBindings();
    autoChooserSetup();
    autoChooserSetup();
  }


  private void registerNamedCommands() {

    NamedCommands.registerCommand("SetArmToPositionCmd",
        new SetArmToPositionCmd(63, m_armSub)); // put whatever number you want in here, I assume its in degrees

    NamedCommands.registerCommand("SetElevatorToHeightCmd",
        new SetElevatorToHeightCmd(74, m_elevatorSub)); // put whatever number you want in here
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    // Drive controller bindings ////////////////////////////////////////////////////////////////////////////////////////////////

    // Square - unused

    m_driverController.cross().whileTrue(m_drivetrainSub.applyRequest(() -> brake));

    m_driverController.circle().whileTrue(m_drivetrainSub
        .applyRequest(() -> point
            .withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))));

    // Triange - unused

    if(isLimelight) {
      m_driverController.L1().onTrue(new AutoDriveCmd(m_visionSub, m_drivetrainSub));

      m_driverController.R1().onTrue(new AutoDriveCmd(m_visionSub, m_drivetrainSub));
    }


    // L2 - unused

    // R2 - unused

    // Share - unused

    // Options - unused

    // Reset the field-centric heading
    m_driverController.PS().onTrue(m_drivetrainSub.runOnce(() -> m_drivetrainSub.seedFieldCentric()));

    // Touchpad - unused

    // 'Kill All' commands
    m_driverController.L3().onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_elevatorSub, m_intakeSub));

    m_driverController.R3().onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_elevatorSub, m_intakeSub));

    // Combination buttons for diagnostics
    // Run SysId routines when holding share/options and square/triangle.
    // Note that each routine should be run exactly once in a single log.
    m_driverController.share().and(m_driverController.triangle())
        .whileTrue(m_drivetrainSub.sysIdDynamic(Direction.kForward));
    m_driverController.share().and(m_driverController.square())
        .whileTrue(m_drivetrainSub.sysIdDynamic(Direction.kReverse));
    m_driverController.options().and(m_driverController.triangle())
        .whileTrue(m_drivetrainSub.sysIdQuasistatic(Direction.kForward));
    m_driverController.options().and(m_driverController.square())
        .whileTrue(m_drivetrainSub.sysIdQuasistatic(Direction.kReverse));


    // Operator Controller Bindings /////////////////////////////////////////////////////////////////////////////////////////////

    // Square - unused

    m_operatorController.triangle().whileTrue(
        new StartEndCommand(() -> m_intakeSub.setRollersPower(1.0), () -> m_intakeSub.setRollersPower(0.0),
            m_intakeSub));

    // Circle - unused


    m_operatorController.cross().onTrue(new InstantCommand(() -> m_elevatorSub.setTargetHeight(100), m_elevatorSub));

    m_operatorController.L1()
        .whileTrue(
            new StartEndCommand(() -> m_climbSub.setPower(1.0), () -> m_climbSub.setPower(0.0), m_climbSub));

    m_operatorController.R1()
        .whileTrue(
            new StartEndCommand(() -> m_climbSub.setPower(-1.0), () -> m_climbSub.setPower(0.0), m_climbSub));


    // Share - unused

    // Options - unused

    // PS - unused

    // Touchpad - unused

    // 'Kill All' commands
    m_driverController.L3().onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_elevatorSub, m_intakeSub));

    m_driverController.R3().onTrue(new KillAllCmd(m_climbSub, m_drivetrainSub, m_elevatorSub, m_intakeSub));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
  }

  void autoChooserSetup() {
    m_Chooser.addOption("Testing Auto", new PathPlannerAuto("Testing Auto"));
    m_Chooser.addOption("DoNothingAuto", new DoNothingGrp());
    SmartDashboard.putData("auto choices", m_Chooser);

  }
}

