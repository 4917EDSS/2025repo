// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.PathGenCmd;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  String path;

  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>
   * If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBaseThread robotBaseThread = new RobotBaseThread();
    final PathGenCmd m_pathGenCmd = new PathGenCmd();
    Thread t1 = new Thread((Runnable) robotBaseThread);
    Thread t2 = new Thread(m_pathGenCmd);
    t1.setPriority(10);
    t2.setPriority(1);
    t1.start();
    t2.start();
    //RobotBase.startRobot(Robot::new);
  }
}
