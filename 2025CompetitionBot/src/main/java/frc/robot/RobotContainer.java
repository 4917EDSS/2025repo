// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import java.util.Arrays;
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
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeRemovalL2L3Grp;
import frc.robot.commands.AlgaeRemovalL3L4Grp;
import frc.robot.commands.ArmMoveWithJoystickCmd;
import frc.robot.commands.AutoDriveCmd;
import frc.robot.commands.DoNothingGrp;
import frc.robot.commands.DriveToNearestScoreLocationCmd;
import frc.robot.commands.ElevatorMoveWithJoystickCmd;
import frc.robot.commands.HomeButton;
import frc.robot.commands.KillAllCmd;
import frc.robot.commands.L2PlacementGrp;
import frc.robot.commands.L3PlacementGrp;
import frc.robot.commands.L4PlacementGrp;
import frc.robot.commands.PlaceReefGrp;
import frc.robot.commands.SetArmToPositionCmd;
import frc.robot.commands.tests.RunTestsGrp;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.CanSub;
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
  boolean m_isLimelight = true;
  // Swerve constants and objects (from CTRE Phoenix Tuner X)
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(2.08 / 2.0).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity, 2.08 is the speed that made it tip over, very funny video
  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveTelemetry swerveLogger = new SwerveTelemetry(MaxSpeed);

  // RobotContainer constants
  private final TestManager m_testManager = new TestManager();

  // Robot subsystems
  //private final ArduinoSub m_arduinoSub = new ArduinoSub();   // TODO:  Implement new CAN Arduino
  private final ArmSub m_armSub = new ArmSub();
  private final ClimbSub m_climbSub = new ClimbSub();
  public final DrivetrainSub m_drivetrainSub = TunerConstants.createDrivetrain();
  private final ElevatorSub m_elevatorSub = new ElevatorSub();
  private final IntakeSub m_intakeSub = new IntakeSub();
  private final CanSub m_canSub = new CanSub(4);
  //private final LedSub m_ledSub = new LedSub(m_arduinoSub);  // TODO:  Implement with new Arduino
  private final VisionSub m_visionSub;

  // Controllers
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final CommandPS4Controller m_operatorController =
      new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);

  // RobotContainer member variables
  public static boolean disableShuffleboardPrint = false;

  SendableChooser<Command> m_Chooser = new SendableChooser<>();

  private final PathConstraints m_constraints = new PathConstraints(
      6.21, 3.0,
      Units.degreesToRadians(360), Units.degreesToRadians(720));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_armSub.setElevatorPositionSupplier(() -> m_elevatorSub.getPositionMm());
    m_elevatorSub.setArmAngleSupplier(() -> m_armSub.getAngle());
    // Only initialize vision if a camera is connected (prevents crash)
    // NetworkTableEvent.Kind[] topicsArray = NetworkTableEvent.Kind.values();
    // if(!Arrays.asList(topicsArray).contains("limelight-left")
    //   && !Arrays.asList(topicsArray).contains("limelight-right")) {
    m_isLimelight = false;
    m_visionSub = null;
    //   System.out.println("No limelights found");
    // } else {
    //m_visionSub = new VisionSub(m_drivetrainSub);
    //System.out.println("Limelight Found");
    //}

    m_testManager.setTestCommand(new RunTestsGrp(m_climbSub, m_armSub, m_elevatorSub, m_intakeSub, m_testManager));

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

    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // Warmup MUST happen before we configure buttons or autos.
    //DriveToNearestScoreLocationCmd.warmUpMap(m_constraints);
    // Configure the trigger bindings
    configureBindings();
    autoChooserSetup();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("SetArmToPositionCmd",
        new SetArmToPositionCmd(63, m_armSub)); // put whatever number you want in here, I assume its in degrees
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    // Drive controller bindings ////////////////////////////////////////////////////////////////////////////////////////////////

    // Square
    m_driverController.square().onTrue(new L3PlacementGrp(m_armSub, m_elevatorSub)); //TODO Make Coral Recieval Command
    // m_driverController.square().whileTrue(AutoBuilder.pathfindToPose(
    //     new Pose2d(2, 6.5, new Rotation2d(0)),
    //     m_constraints,
    //     0.0 // Goal end velocity in meters/sec
    // ));

    //Triangle - L4 Coral Placement
    m_driverController.triangle().onTrue(new L4PlacementGrp(m_armSub, m_elevatorSub));

    // Cross - L2 Coral Placement
    //m_driverController.cross().whileTrue(m_drivetrainSub.applyRequest(() -> brake));
    m_driverController.cross().onTrue(new L2PlacementGrp(m_armSub, m_elevatorSub));

    // Circle - L3 Coral Placement
    m_driverController.circle().onTrue(new L3PlacementGrp(m_armSub, m_elevatorSub));
    // m_driverController.circle().whileTrue(m_drivetrainSub
    //     .applyRequest(() -> point
    //         .withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))));

    //L1 - Remove L2-L3 Algae
    m_driverController.L1().onTrue(new AlgaeRemovalL2L3Grp(m_armSub, m_elevatorSub));

    //R1 - Remove L3-L4 Algae
    m_driverController.R1().onTrue(new AlgaeRemovalL3L4Grp(m_armSub, m_elevatorSub));

    // if(m_isLimelight) {
    //   // L1
    //   m_driverController.L1().onTrue(new AutoDriveCmd(m_visionSub, m_drivetrainSub));
    //   // R1
    //   m_driverController.R1().onTrue(new HomeButton(m_armSub, m_elevatorSub));
    //   //new AutoDriveCmd(m_visionSub, m_drivetrainSub));
    // }

    // L2
    // m_driverController.L2().onTrue(new InstantCommand(() -> m_armSub.setTargetAngle(0), m_armSub));
    //m_driverController.L2().onTrue(new SetArmToPositionCmd(30, m_armSub));

    // R2S
    //m_driverController.R2().onTrue(new SetArmToPositionCmd(0, m_armSub));
    // m_driverController.R2().onTrue(new InstantCommand(() -> m_armSub.setTargetAngle(45), m_armSub));

    // Share

    // Options

    // PS
    // Reset the field-centric heading
    m_driverController.PS().onTrue(m_drivetrainSub.runOnce(() -> m_drivetrainSub.seedFieldCentric()));

    //L3

    //R3

    //Pov up - Move climb up to a set position
    //m_driverController.povUp().onTrue() // TODO add command to raise and lower climb.
    //Pov down - Move Climb down to a set position
    //m_driverController.povDown().onTrue() 

    // Touchpad
    m_driverController.touchpad()
        .onTrue(new KillAllCmd(m_armSub, m_climbSub, m_drivetrainSub, m_elevatorSub, m_intakeSub));

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

    // Square
    m_operatorController.square().onTrue(new L3PlacementGrp(m_armSub, m_elevatorSub));

    // Cross
    m_operatorController.cross().onTrue(new InstantCommand(() -> m_elevatorSub.setTargetHeight(900), m_elevatorSub));

    // Circle
    m_operatorController.circle().whileTrue(new L4PlacementGrp(m_armSub, m_elevatorSub));

    // Triangle
    m_operatorController.triangle().whileTrue(new StartEndCommand(() -> m_intakeSub.setRollersPower(1.0),
        () -> m_intakeSub.setRollersPower(0.0), m_intakeSub));

    // L1
    m_operatorController.L1()
        .whileTrue(new StartEndCommand(() -> m_climbSub.setPower(0.10), () -> m_climbSub.setPower(0.0), m_climbSub));

    // R1
    m_operatorController.R1()
        .whileTrue(new StartEndCommand(() -> m_climbSub.setPower(-0.10), () -> m_climbSub.setPower(0.0), m_climbSub));

    // L2
    m_operatorController.L2().onTrue(new AlgaeRemovalL3L4Grp(m_armSub, m_elevatorSub));

    // R2
    m_operatorController.R2().whileTrue(new AlgaeRemovalL2L3Grp(m_armSub, m_elevatorSub));

    // Share
    m_operatorController.share().onTrue(new HomeButton(m_armSub, m_elevatorSub));

    // Options
    m_operatorController.options().onTrue(new L2PlacementGrp(m_armSub, m_elevatorSub));

    // PS
    m_operatorController.PS().onTrue(new InstantCommand(() -> {
      m_elevatorSub.setPositionMm(0);
      m_elevatorSub.setTargetHeight(0);
    }, m_elevatorSub));

    // Touchpad

    // L3
    m_operatorController.L3().onTrue(new KillAllCmd(m_armSub, m_climbSub, m_drivetrainSub, m_elevatorSub, m_intakeSub));

    // R3
    m_operatorController.R3().onTrue(new KillAllCmd(m_armSub, m_climbSub, m_drivetrainSub, m_elevatorSub, m_intakeSub));
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
    m_Chooser.addOption("Leave Auto", new PathPlannerAuto("Leave Auto"));
    m_Chooser.addOption("DoNothingAuto", new DoNothingGrp());
    SmartDashboard.putData("auto choices", m_Chooser);
  }
}
