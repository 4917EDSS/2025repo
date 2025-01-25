// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final int kArmLowerLimit = 2;
    public static final int kArmUpperLimit = 3;

    public static final int kIntakeLowerLimit = 4;
    public static final int kIntakeUpperLimit = 5;
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

  public final static class Arm {
    public static final double kEncoderPositionConversionFactor = 1.00; // From rotations to degrees
    public static final double kEncoderVelocityConversionFactor = 1.00; // From rotations per minute? to degrees per second
    public static final double kMinArmAngle = 0.0; // In degrees
    public static final double kMaxArmAngle = 170.0; // In degrees
  }

  public final static class Climb {

  }

  public final static class Elevator {
    public final static double kMinHeight = 0.0; // In mm
    public final static double kMaxHeight = 2000.0; // In mm
    // Sets a max power if we are close to the lower limit switch
    public final static double kSlowDownLowerStagePower = -0.25;
    public final static double kSlowDownLowerStageHeight = 20.0;
  }

  public static class Intake {
    public static final double kDeployEncoderPositionConversionFactor = 1.00; // From rotations to degrees
    public static final double kDeployEncoderVelocityConversionFactor = 1.00; // From rotations per minute? to degrees per second

    public static final double kRollersEncoderPositionConversionFactor = 1.00; // From rotations to degrees
    public static final double kRollersEncoderVelocityConversionFactor = 1.00; // From rotations per minute? to degrees per second
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

    public static final double kDriveMotorPower = 0.1; // Power to test motor at
    public static final long kDriveMotorTimeMs = 2000; // How long to run the motor for, in milliseconds
    public static final double kDriveMotorExpectedPosition = 31; // What the expected encoder position is (in ? units)
    public static final double kDriveMotorPositionTolerance = 0.75; // How for off the position can be (+ or -) and still considered OK
    public static final double kDriveMotorPositionMinimum = 1; // Minimum position change to be considered a partial pass
    public static final double kDriveMotorExpectedAmps = 0.06; // How much current we expect it to pull just before the end of the test
    public static final double kDriveMotorAmpsTolerance = 0.03; // How far off the amps can be (+ or -) and still be considered OK
    public static final double kDriveMotorAmpsMinimum = 0.02; // Minimum current to be considered a partial pass

    public static final double kIntakeMotorPower = 0.1; // Power to test motor at
    public static final long kIntakeMotorTimeMs = 2000; // How long to run the motor for, in milliseconds
    public static final double kIntakeMotorExpectedPosition = 31; // What the expected encoder position is (in ? units)
    public static final double kIntakeMotorPositionTolerance = 0.75; // How for off the position can be (+ or -) and still considered OK
    public static final double kIntakeMotorPositionMinimum = 1; // Minimum position change to be considered a partial pass
    public static final double kIntakeMotorExpectedAmps = 0.06; // How much current we expect it to pull just before the end of the test
    public static final double kIntakeMotorAmpsTolerance = 0.03; // How far off the amps can be (+ or -) and still be considered OK
    public static final double kIntakeMotorAmpsMinimum = 0.02; // Minimum current to be considered a partial pass
  }

}
