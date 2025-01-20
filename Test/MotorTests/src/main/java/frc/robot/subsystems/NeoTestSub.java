// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class NeoTestSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(NeoTestSub.class.getName());
  private final SparkMax m_neoVortexMotor = new SparkMax(Constants.CanIds.kNeoVortexId, MotorType.kBrushless);
  //private final SparkFlex m_neoVortexMotor = new SparkFlex(Constants.CanIds.kNeoVortexId, MotorType.kBrushless);

  private final ArmFeedforward m_armFeedforward = new ArmFeedforward(0.001, 0.001, 0.0);
  private final PIDController m_PID_NEOController = new PIDController(0.01, 0.001, 0.001); // really needs some tuning
  private final double m_arm_Velocity = 70.0;
  private double m_targetAngle = 0;

  private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("PID Tuning");
  private final GenericEntry m_neoVortexMotorVelocity, m_newVortexMotorAngle;

  /** Creates a new NeoTestSub. */
  public NeoTestSub() {

    m_neoVortexMotorVelocity = m_shuffleboardTab.add("NEO Velocity", 0).getEntry();

    m_newVortexMotorAngle = m_shuffleboardTab.add("NEO Angle", 0).getEntry();


    init();
  }

  public void init() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
        .inverted(false)
        .smartCurrentLimit(5)
        .idleMode(IdleMode.kCoast);

    config.encoder
        .positionConversionFactor(Constants.NeoSub.kRotationsToAngleFactor);

    // Only persist parameters when configuring the motor on start up as this operation can be slow
    m_neoVortexMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    // If we change the parameters (e.g. brake mode) during robot operation, 
    // we should not save the changes to flash (i.e. want kNoPersistParameters)
    //m_neoVortexMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    resetEncoder();
  }

  @Override
  public void periodic() {
    updateShuffleBoard();
    SmartDashboard.putNumber("Encoder", getAngle());

    runPIDControl();
    // This method will be called once per scheduler run

    // double feedForwardVoltage = m_armFeedforward.calculate(m_arm_Velocity, 0.0);
    // accelaration has been depreciated in 2025 WPILIB.
    // double feedForwardVoltage = m_armFeedforward.calculate(m_arm_Velocity);
    // double Feedforward = m_armFeedforward.calculate(m_arm_Velocity, new SIUnit<Acceleration> 0.0);

    // So far, we don't need the PID control.  Feedforward is doing well on its own
    // double pidVoltage = m_PID_NEOController.calculate(getArmVelocity(), m_arm_Velocity);

    // setArmVoltage(feedForwardVoltage + pidVoltage);
  }

  public double getAngle() {
    return m_neoVortexMotor.getEncoder().getPosition();
  }

  public void setAngle(double targetAngle) {
    m_targetAngle = targetAngle;
  }

  public void setPower(double power) {
    m_neoVortexMotor.set(power);
  }

  public void resetEncoder() {
    m_logger.warning("Resetting Sparkmax encoder");
    m_neoVortexMotor.getEncoder().setPosition(0.0);
  }

  public double getArmVelocity() {
    return m_neoVortexMotor.getEncoder().getVelocity();
  }

  public void setArmVoltage(double voltage) {
    m_neoVortexMotor.setVoltage(voltage);

  }

  private void updateShuffleBoard() {
    m_neoVortexMotorVelocity.setDouble(getArmVelocity());


    m_newVortexMotorAngle.setDouble(getAngle());


  }

  public void runPIDControl() {
    double pidPower = m_PID_NEOController.calculate(getAngle(), m_targetAngle);
    double fedPower = m_armFeedforward.calculate(Math.toRadians(getAngle()), pidPower); // Feed forward expects 0 degrees as horizontal


    double pivotPower = pidPower + fedPower;
    setPower(pivotPower);

  }
}
