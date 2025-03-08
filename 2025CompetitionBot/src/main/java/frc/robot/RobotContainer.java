// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmMoveWithJoystickCmd;
import frc.robot.commands.AutoAlgaeRemovalL2L3Grp;
import frc.robot.commands.AutoAlgaeRemovalL3L4Grp;
import frc.robot.commands.AutoCoralScoreL2Grp;
import frc.robot.commands.AutoCoralScoreL3Grp;
import frc.robot.commands.AutoCoralScoreL4Grp;
import frc.robot.commands.AutoDriveCmd;
import frc.robot.commands.BackUpAfterScoringCmd;
import frc.robot.commands.DoNothingGrp;
import frc.robot.commands.DriveToNearestScoreLocationCmd;
import frc.robot.commands.ElevatorMoveWithJoystickCmd;
import frc.robot.commands.AutoGrabCoralGrp;
import frc.robot.commands.KillAllCmd;
import frc.robot.commands.MoveElArmGrp;
import frc.robot.commands.SetArmToPositionCmd;
import frc.robot.commands.SetElevatorToHeightCmd;
import frc.robot.commands.tests.RunTestsGrp;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.CanSub;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.ElevatorSub;
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
  boolean m_isLimelight = true;
  // Swerve constants and objects (from CTRE Phoenix Tuner X)
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(2.08 / 2.0).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity, 2.08 is the speed that made it tip over, very funny video
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 1% deadband
      .withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveTelemetry swerveLogger = new SwerveTelemetry(MaxSpeed);

  // RobotContainer constants
  private final TestManager m_testManager = new TestManager();

  // Robot subsystems
  private final CanSub m_canSub = new CanSub(Constants.CanIds.kElevatorCustomCanBoard);
  private final ArmSub m_armSub = new ArmSub(m_canSub);
  private final ClimbSub m_climbSub = new ClimbSub();
  private final DrivetrainSub m_drivetrainSub = TunerConstants.createDrivetrain();
  private final ElevatorSub m_elevatorSub = new ElevatorSub();
  private final VisionSub m_visionSub;

  // Controllers
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final CommandPS4Controller m_operatorController =
      new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);

  // RobotContainer member variables
  public static boolean disableShuffleboardPrint = false;
  private SendableChooser<Command> m_Chooser = new SendableChooser<>();
  private final PathConstraints m_constraints = new PathConstraints(
      6.21, 3.0,
      Units.degreesToRadians(360), Units.degreesToRadians(720));


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // turn off logging
    SignalLogger.stop();
    m_armSub.setElevatorPositionSupplier(() -> m_elevatorSub.getPositionMm());
    m_elevatorSub.setArmAngleSupplier(() -> m_armSub.getAngle());

    m_visionSub = new VisionSub(m_drivetrainSub); // TODO:  Check if camera is present before initializing this or move it back up with the other subsystem inits

    m_testManager.setTestCommand(new RunTestsGrp(m_climbSub, m_armSub, m_elevatorSub, m_testManager));

    // Default commands
    m_drivetrainSub.setDefaultCommand(
        // Note: X is defined as forward and Y as left according to WPILib convention
        m_drivetrainSub.applyRequest(() -> drive
            .withVelocityX(-Math.abs(m_driverController.getLeftY()) * m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-Math.abs(m_driverController.getLeftX()) * m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    m_armSub.setDefaultCommand(new ArmMoveWithJoystickCmd(m_operatorController, m_armSub));
    m_elevatorSub.setDefaultCommand(new ElevatorMoveWithJoystickCmd(m_operatorController, m_elevatorSub));

    // Register Swerve telemetry
    //m_drivetrainSub.registerTelemetry(swerveLogger::telemeterize);

    // Start a webserver that Elastic dashboard can use to pull the saved dashboard from the computer
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // Warmup MUST happen before we configure buttons or autos.
    DriveToNearestScoreLocationCmd.warmUpMap(m_constraints);

    configureBindings();
    autoChooserSetup();
    registerNamedCommands();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("SetArmToPositionCmd 63",
        new SetArmToPositionCmd(63, m_armSub)); // put whatever number you want in here, I assume its in degrees

    NamedCommands.registerCommand("SetElevatorToHeightCmd 100",
        new SetElevatorToHeightCmd(100, m_elevatorSub)); // put whatever number you want in here. probably mm

    NamedCommands.registerCommand("BackUpAfterScoringCmd",
        new BackUpAfterScoringCmd(m_drivetrainSub, m_constraints));
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    // Drive controller bindings ////////////////////////////////////////////////////////////////////////////////////////////////

    // Square

    //m_driverController.square().onTrue(new GrabCoralGrp(m_armSub, m_canSub, m_elevatorSub));
    m_driverController.square().whileTrue(new AutoDriveCmd(m_visionSub, m_drivetrainSub, 0.22));


    // Cross
    m_driverController.cross().onTrue(new AutoCoralScoreL2Grp(m_armSub, m_elevatorSub)); // TODO: Implement this command group

    // Circle
    m_driverController.cross()
        .onTrue(new AutoCoralScoreL3Grp(0.22, m_armSub, m_canSub, m_drivetrainSub, m_elevatorSub, m_visionSub));

    // Triangle
    m_driverController.cross()
        .onTrue(new AutoCoralScoreL4Grp(0.22, m_armSub, m_canSub, m_drivetrainSub, m_elevatorSub, m_visionSub));
    // L1
    m_driverController.L1().onTrue(new AutoAlgaeRemovalL2L3Grp(m_armSub, m_elevatorSub));

    // R1
    m_driverController.R1().onTrue(new AutoAlgaeRemovalL3L4Grp(m_armSub, m_elevatorSub));

    // L2
    m_driverController.L2().onTrue(new InstantCommand(() -> slowDown())).onFalse(new InstantCommand(() -> speedUp()));

    // R2

    // POV Up
    // TODO add command move the climb arm to the climb position

    // POV Right
    // TODO:  Target scoring to pipe to the right of the vision target

    // POV Down
    // TODO Move Climb arm in to climb

    // POV Left
    // TODO:  Target scoring to pipe to the left of the vision target

    // Share
    m_driverController.share().onTrue(new BackUpAfterScoringCmd(m_drivetrainSub, m_constraints));

    // Options
    m_driverController.options().whileTrue(AutoBuilder.pathfindToPose(
        new Pose2d(2, 6.5, new Rotation2d(0)),
        m_constraints,
        0.0 // Goal end velocity in meters/sec
    ));

    // PS
    m_driverController.PS().onTrue(m_drivetrainSub.runOnce(() -> m_drivetrainSub.seedFieldCentric())); // Reset the field-centric heading

    // L3

    // R3

    // Touchpad
    m_driverController.touchpad()
        .onTrue(new KillAllCmd(m_armSub, m_canSub, m_climbSub, m_drivetrainSub, m_elevatorSub));

    // Combination buttons for diagnostics
    // Run SysId routines when holding share/options and square/triangle.
    // Note that each routine should be run exactly once in a single log.
    // m_driverController.share().and(m_driverController.triangle())
    //     .whileTrue(m_drivetrainSub.sysIdDynamic(Direction.kForward));
    // m_driverController.share().and(m_driverController.square())
    //     .whileTrue(m_drivetrainSub.sysIdDynamic(Direction.kReverse));
    // m_driverController.options().and(m_driverController.triangle())
    //     .whileTrue(m_drivetrainSub.sysIdQuasistatic(Direction.kForward));
    // m_driverController.options().and(m_driverController.square())
    //     .whileTrue(m_drivetrainSub.sysIdQuasistatic(Direction.kReverse));


    // Operator Controller Bindings /////////////////////////////////////////////////////////////////////////////////////////////

    // Square
    m_operatorController.square().onTrue(new AutoGrabCoralGrp(m_armSub, m_canSub, m_elevatorSub));

    // Cross
    m_operatorController.cross().onTrue(new MoveElArmGrp(Constants.Elevator.kL2PreScoreHeight,
        Constants.Arm.kL2PreScoreAngle, m_armSub, m_elevatorSub));

    // Circle
    m_operatorController.circle().onTrue(new MoveElArmGrp(Constants.Elevator.kL3PreScoreHeight,
        Constants.Arm.kL3PreScoreAngle, m_armSub, m_elevatorSub));

    // Triangle
    m_operatorController.triangle().onTrue(new MoveElArmGrp(Constants.Elevator.kL4PreScoreHeight,
        Constants.Arm.kL4PreScoreAngle, m_armSub, m_elevatorSub));

    // L1
    m_operatorController.L1()
        .onTrue(new MoveElArmGrp(Constants.Elevator.kL2L3AlgaeRemovalPrepHeight,
            Constants.Arm.kL2L3AlgaeRemovalPrepAngle, m_armSub, m_elevatorSub));

    // R1
    m_operatorController.R1()
        .whileTrue(new MoveElArmGrp(Constants.Elevator.kL3L4AlgaeRemovalPrepHeight,
            Constants.Arm.kL3L4AlgaeRemovalPrepAngle, m_armSub, m_elevatorSub));

    // L2
    // TODO: Remove algae based on which one we are prepped for

    // R2
    // TODO: Score coral based on which one we are prepped for

    // POV Up
    m_operatorController.povUp()
        .whileTrue(new StartEndCommand(() -> m_climbSub.setPower(0.25), () -> m_climbSub.setPower(0.0), m_climbSub));

    // POV Right

    // POV Down
    m_operatorController.povDown()
        .whileTrue(new StartEndCommand(() -> m_climbSub.setPower(-0.25), () -> m_climbSub.setPower(0.0), m_climbSub));

    // POV Left

    // Share
    m_operatorController.share().onTrue(new InstantCommand(() -> m_armSub.setTargetAngle(-75), m_armSub));

    // Options
    m_operatorController.options().onTrue(new InstantCommand(() -> m_armSub.setTargetAngle(25), m_armSub));

    // PS
    m_operatorController.PS().onTrue(new InstantCommand(() -> m_elevatorSub.allowEncoderReset(), m_elevatorSub));

    // Touchpad
    m_operatorController.touchpad()
        .onTrue(new KillAllCmd(m_armSub, m_canSub, m_climbSub, m_drivetrainSub, m_elevatorSub));

    // L3
    m_operatorController.L3().onTrue(new KillAllCmd(m_armSub, m_canSub, m_climbSub, m_drivetrainSub, m_elevatorSub));

    // R3
    m_operatorController.R3().onTrue(new KillAllCmd(m_armSub, m_canSub, m_climbSub, m_drivetrainSub, m_elevatorSub));
  }

  /**
   * Lower the max drive speed for precision driving
   */
  private void slowDown() {
    MaxSpeed = MaxSpeed * 0.1;
    MaxAngularRate = MaxAngularRate * 0.1;
  }

  /**
   * Restore the max drive speed for precision driving
   */
  private void speedUp() {
    MaxSpeed = MaxSpeed * 10;
    MaxAngularRate = MaxAngularRate * 10;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
  }

  /**
   * Create a list of auto period action choices
   */
  void autoChooserSetup() {
    m_Chooser.addOption("Leave", new PathPlannerAuto("Leave Auto"));
    m_Chooser.addOption("Do-Nothing", new DoNothingGrp());
    SmartDashboard.putData("Auto Choices", m_Chooser);
  }
}
