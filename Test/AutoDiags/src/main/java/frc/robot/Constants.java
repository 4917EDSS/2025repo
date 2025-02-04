// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.utils.TestMotorParameters;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CandIds {
    public static final int kElevatorMotor1 = 1;
    public static final int kElevatorMotor2 = 2;
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
    public static final TestMotorParameters kElevatorMotor1 =
        new TestMotorParameters("ElevatorM1", 1, 0.1, 1000, 75, 3.0, 10, 1.5, 0.1, 0.5);
    public static final TestMotorParameters kElevatorMotor2 =
        new TestMotorParameters("ElevatorM2", 2, 0.1, 1000, 75, 3.0, 10, 1.5, 0.1, 0.5);
  }
}
