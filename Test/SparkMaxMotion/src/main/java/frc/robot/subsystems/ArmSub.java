// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSub extends SubsystemBase {
  private final SparkMax m_armMotor = new SparkMax(1, MotorType.kBrushless);
  private final SparkAbsoluteEncoder m_absoluteEncoder = m_armMotor.getAbsoluteEncoder();
  private final SparkClosedLoopController m_controller = m_armMotor.getClosedLoopController();

  private double m_kMinOutput = -0.25; // -1.0 to 1.0
  private double m_kMaxOutput = 0.25; // -1.0 to 1.0
  private double m_kMaxVel = 1; // RPM
  private double m_kMaxAccel = 1; // RPM/s
  private double m_kP = 0.0;
  private double m_kI = 0.0;
  private double m_kD = 0.0;
  private double m_kAllowedError = 0.1; // at-position tolerance, in revolutions

  private boolean m_automationEnabled = false;
  private double m_targetAngle = 0;
  private int m_printCount = 0;

  /** Creates a new ArmSub. */
  public ArmSub() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(false) // Set to true to invert the forward motor direction
        .smartCurrentLimit(60) // Current limit in amps
        .idleMode(IdleMode.kBrake)
            // Set PID gains
            .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(m_kP)
                .i(m_kI)
                .d(m_kD)
                .outputRange(m_kMinOutput, m_kMaxOutput)
                    // Set MAXMotion parameters
                    .maxMotion
                        .maxVelocity(m_kMaxVel)
                        .maxAcceleration(m_kMaxAccel)
                        .allowedClosedLoopError(m_kAllowedError);

    // Save the configuration to the motor
    // Only persist parameters when configuring the motor on start up as this
    // operation can be slow
    m_armMotor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    if(++m_printCount >= 10) {
      m_printCount = 0;
      System.out.println("Pwr=" + m_armMotor.get()
          + " Pos=" + getPosition()
          + " Vel=" + getVelocity()
          + " Trg=" + m_targetAngle
          + " Auto=" + (m_automationEnabled ? "Y" : "N"));
    }
  }

  public void setPower(double power) {
    m_armMotor.set(power);
  }

  public void setTargetAngle(double angle) {
    m_targetAngle = angle;
    m_controller.setReference(m_targetAngle, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  public double getPosition() {
    return m_absoluteEncoder.getPosition(); // returns rotations
  }

  public double getVelocity() {
    return m_absoluteEncoder.getVelocity();
  }

  public void enableAutomation() {
    m_automationEnabled = true;
  }

  public void disableAutomation() {
    m_automationEnabled = false;
  }
}
