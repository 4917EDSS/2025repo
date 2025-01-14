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

  public static class CanIds {
    public static final int kElevatorMotor = 9;
    public static final int kClimbMotor = 10; 
    public static final int kIntakeMotor = 11;
    public static final int kKrakenMotor = 1;
    public static final int kKrakenMotor2 = 2;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public final static class Arduino {
    public static final int kBaudRate = 38400;
    public static final double kTimeOutLength = 0.0;
    public static final int kReadMessageLength = 19;
    public static final int kBufferSize = kReadMessageLength * 2;
    public static final int kSensorDataLength = 16;
    public static final byte kMessageHeader = (byte) 0xA5;
  }

  public final static class PwmIds {
    public final static int kLedStripPwmPort = 0;
  }

  // Subsystem Constants
  public final static class Vision {
    //TODO: change apriltag heights to actual heights, as well as the offsets. This is from last year.
    public static final double kApriltagOffset = 0.0825; // Apriltag height + bot height (Will need to be changed in the future)
    public static final double kApriltagHeights[] =
        {1.22, 1.22, 1.32, 1.32, 1.22, 1.22, 1.32, 1.32, 1.22, 1.22, 1.24, 1.24, 1.24, 1.24, 1.24, 1.24};
    }

    public static final class Tests {
      public static final String kTabName = "Tests";
      public static final int kDashboardRows = 5; // Max rows that we can use to display tests (start new column after this row)
      public static final int kDashboardCols = 6; // Max columns that we cna use to display tests (start a new tab after this column)
  
      public static final double kDriveMotorPower = 0.1; // Power to test motor at
      public static final long kDriveMotorTimeMs = 2000; // How long to run the motor for, in milliseconds
      public static final double kDriveMotorExpectedPosition = 31; // What the expected encoder position is (in ? units)
      public static final double kDriveMotorPositionTolerance = 0.75; // How for off the position can be (+ or -) and still considered OK
      public static final double kDriveMotorPositionMinimum = 1; // Minimum position change to be considered a partial pass
      public static final double kDriveMotorExpectedAmps = 0.06; // How much current we expect it to pull just before the end of the test
      public static final double kDriveMotorAmpsTolerance = 0.03; // How far off the amps can be (+ or -) and still be considered OK
      public static final double kDriveMotorAmpsMinimum = 0.02; // Minimum current to be considered a partial pass
    }
}
