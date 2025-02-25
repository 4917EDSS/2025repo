// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.logging.Level;
import edu.wpi.first.units.measure.Angle;
import frc.robot.utils.TestMotorParameters;
import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Max log level to print (SEVERE, WARNING, INFO, CONFIG, FINE, FINER, or FINEST)
  // e.g. Level.WARNING will only print WARNING and SEVERE log messages
  public static final Level kLogLevel = Level.FINE;

  ////////// Hardware mapping /////////////////////////////////////////////////////////////////////
  public static class CanIds {
    // These are the roboRIO CAN bus IDs
    // CTRE Swerve drivetrain uses CAN IDs 1-13 on CANivore bus
    // This does not conflict with the roboRIO bus which can also use these IDs
    public static final int kElevatorMotor = 1;
    public static final int kElevatorMotor2 = 2;
    public static final int kArmMotor = 3;
    public static final int kIntakeDeployMotor = 4;
    public static final int kIntakeRollersMotor = 5;
    public static final int kClimbMotor = 6;
  }

  public static class DioIds {
    public static final int kElevatorLowerLimit = 0;
    public static final int kElevatorUpperLimit = 1;
    public static final int kElevatorEncoderResetSwitch = 2;
    public static final int kElevatorStageTwoLimit = 3;

    public static final int kIntakeLowerLimit = 6;
    public static final int kIntakeUpperLimit = 7;

    public final static int kClimbInLimitSwitch = 8;
    public final static int kClimbOutLimitSwitch = 9;
  }

  public final static class PwmIds {
    public final static int kLedStripPwmPort = 0;
  }


  ////////// Subsystem constants /////////////////////////////////////////////////////////////////////
  public final static class Arduino {
    public static final int kBaudRate = 38400;
    public static final double kTimeOutLength = 0.0;
    public static final int kReadMessageLength = 19;
    public static final int kBufferSize = kReadMessageLength * 2;
    public static final int kSensorDataLength = 16;
    public static final byte kMessageHeader = (byte) 0xA5;
  }

  public static final class Breakers {
    // TODO: All values are from last year. Update to current bot's values
    public static final int kDriveMotorFrontRight = 4; //6
    public static final int kDriveMotorFrontLeft = 11; //19
    public static final int kDriveMotorBackRight = 6; //4
    public static final int kDriveMotorBackLeft = 1; //0
    public static final int kSteeringMotorFR = 5; //5
    public static final int kSteeringMotorFL = 14; //10
    public static final int kSteeringMotorBR = 7; //3
    public static final int kSteeringMotorBL = 15; //18

    public static final int kClimbMotorL = 3; //1
    public static final int kClimbMotorR = 0; //2

    public static final int kLowerFeeder = 12; //12
    public static final int kUpperFeeder = 9; //11
    public static final int kPivot = 13; //17

    public static final int kFlywheelL = 2; //8
    public static final int kFlywheelR = 8; //7

    public static final int kIntakeRollers = 10; //9

    public static final int kLimelightUp = 19; // Not on practice bot
    public static final int kLimelightDown = 18; // Not on practice bot
    public static final int kCancoders = 20; // Not on practice bot //they are on the vrm
    public static final int kVRM = 20; //20
    public static final int kRoboRio = 21; //21
    public static final int kArduino = 23; //23 
    public static final int kRadio = 22; //22


  }


  public final static class Arm {
    public static final double kEncoderPositionConversionFactor = 0.01;//1 / 100 * 360; // From rotations to degrees (Gear Ration / 360 deg)
    public static final double kEncoderVelocityConversionFactor = 1.00; // From rotations per minute? to degrees per second
    public static final double kAbsoluteEncoderOffset = 0.0;//practice bot 0.65 // From range to 0 - 1
    public static final double kMinArmAngle = -92.0; // In degrees
    public static final double kMaxArmAngle = 30.0; // In degrees
    public static final double kMaxPower = 0.1;
    public static final double kTargetAngleDeadband = 2.0; // In degrees
    public static final double kAngleTolerance = 5; // In degrees
    public static final double kAtTargetMaxVelocity = 0.02; // In degrees per second?
  }

  public final static class Climb {

  }

  public final static class DriveTrain {
    public static final double kPathingConfig = 1.0; // this is temporary, please change this once it is being used, i have no clue if this works properly
  }

  public final static class Elevator {
    public final static double kMinHeight = 0.0; // In mm
    public final static double kMaxHeight = 1140.0; // In mm

    // Sets a max power if we are close to the lower limit switch
    public final static double kSlowDownLowerStagePower = -0.1;
    public final static double kSlowDownLowerStageHeight = 20.0; // In mm
    public final static double kSlowDownUpperStagePower = 0.1;
    public final static double kSlowDownUpperStageHeight = 900.0; // In mm
    public final static double kTargetHeightDeadbandMM = 5.0; // In mm

    public final static double kRotationsToMm = 5 / (0.75 * 25.4 * Math.PI); // Gearing / Spool diameter in inches * mm/in * PI
    public final static double kStartingHeight = 440.0; // In mm
    public final static double kResetHeight = 450.0; // In mm

    public final static double kHeightTolerance = 5; // Height tolerance for elevator position

    //Sets max power for isElevatorAtTargetHeight function
    public final static double kAtTargetMaxVelocity = 150;
  }

  public static class Intake {
    public static final double kDeployEncoderPositionConversionFactor = 1.00; // From rotations to degrees
    public static final double kDeployEncoderVelocityConversionFactor = 1.00; // From rotations per minute? to degrees per second

    public static final double kRollersEncoderPositionConversionFactor = 1.00; // From rotations to degrees
    public static final double kRollersEncoderVelocityConversionFactor = 1.00; // From rotations per minute? to degrees per second
  }


  // Values that are specific to a particular physical robot
  public static final class RobotSpecific {
    public static final String PracticeSerialNumber = "03147322";
    public static final String CompetitionSerialNumber = "03264244";
    public static final String serialNumber = System.getenv("serialnum");

    public static final class Practice {
      public static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.318115234375);
      public static final Angle kFrontRightEncoderOffset = Rotations.of(0.023193359375);
      public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.064453125);
      public static final Angle kBackRightEncoderOffset = Rotations.of(0.025146484375);
      public static final double kGearRatio = 6.12; //from last year
      public static final double kDriveGearRatio = 5.142857142857142;
      public static final double kSteerGearRatio = 12.8;
      public static final boolean kInvertLowerFeeder = false;
    }

    public static final class Competition {
      public static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.37744140625);
      public static final Angle kFrontRightEncoderOffset = Rotations.of(-0.03759765625);
      public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.148193359375);
      public static final Angle kBackRightEncoderOffset = Rotations.of(-0.4365234375);
      public static final boolean kInvertLowerFeeder = true;
      public static final double kGearRatio = 6.05; //from last year
      public static final double kDriveGearRatio = 5.142857142857142;
      public static final double kSteerGearRatio = 12.8;
    }

    public static final class Unknown {
      public static final Angle kAbsoluteEncoderOffsetFL = Rotations.of(0);
      public static final Angle kAbsoluteEncoderOffsetFR = Rotations.of(0);
      public static final Angle kAbsoluteEncoderOffsetBL = Rotations.of(0);
      public static final Angle kAbsoluteEncoderOffsetBR = Rotations.of(0);
      public static final boolean kInvertLowerFeeder = true;
      public static final double kGearRatio = 0.0;
    }
  }


  public final static class Vision {
    //TODO: change apriltag heights to actual heights, as well as the offsets. This is from last year.
    public static final double kApriltagOffset = 0.0825; // Apriltag height + bot height (Will need to be changed in the future)
    public static final double kApriltagHeights[] =
        {1.22, 1.22, 1.32, 1.32, 1.22, 1.22, 1.32, 1.32, 1.22, 1.22, 1.24, 1.24, 1.24, 1.24, 1.24, 1.24};
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  ////////// Test pass/fail/warn parameters ///////////////////////////////////////////////////////
  public static final class Tests {
    public static final String kTabName = "Tests";
    public static final int kDashboardRows = 5; // Max rows that we can use to display tests (start new column after this row)
    public static final int kDashboardCols = 6; // Max columns that we can use to display tests (start a new tab after this column)

    /*
     * TestMotorParameters are, in order:
     * 
     * testName - Short test title (<12 characters, ideally)
     * motorId - ID of motor to access in the subsystem (starting at 1)
     * power - Power to run motor at, -1.0 to 1.0
     * timeMs - Time test should run for before sampling the results
     * positionMin - Minimum position that motor should pass in order to not be a complete fail
     * positionTarget - Position that the motor is expected to get to +/- the tolerance
     * positionTolerance - Tolerance around the position target that's considered a pass
     * ampsMin - Minimum current (in amps) that motor should draw in order to not be a complete fail
     * ampsTarget - Current that the motor is expected to draw +/- the tolerance
     * ampsTolerance - Tolerance around the current target that's considered a pass
     */

    /* Elevator Motors */
    public static final TestMotorParameters kElevatorMotor1 =
        new TestMotorParameters("ElevatorMotor1", 1, 0.1, 1000, 75, 74, 10, 1.5, 20, 5);
    public static final TestMotorParameters kElevatorMotor2 =
        new TestMotorParameters("ElevatorMotor2", 2, 0.1, 1000, 75, 74, 10, 1.5, 20, 5);
    /* Intake Motors */
    public static final TestMotorParameters kIntakeMotor1 =
        new TestMotorParameters("IntakeMotor1", 1, 0.1, 1000, 75, 3.0, 10, 1.5, 0.1, 0.5);
    public static final TestMotorParameters kIntakeMotor2 =
        new TestMotorParameters("IntakeMotor2", 1, 0.1, 1000, 75, 3.0, 10, 1.5, 0.1, 0.5);
    /* Arm Motor */
    public static final TestMotorParameters kArmMotor =
        new TestMotorParameters("ArmMotor", 1, 0.1, 1000, 75, 3.0, 10, 1.5, 0.1, 0.5);
    /* Climb Motor */
    public static final TestMotorParameters kClimbMotor =
        new TestMotorParameters("ClimbMotor", 1, 0.1, 1000, 75, 3.0, 10, 1.5, 0.1, 0.5);
  }


}
