// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.utils.TestableSubsystem;


public class IntakeSub extends TestableSubsystem {
  private static Logger m_logger = Logger.getLogger(IntakeSub.class.getName());
  private final SparkMax m_intakeDeployMotor = new SparkMax(Constants.CanIds.kIntakeDeployMotor, MotorType.kBrushless);
  private final SparkMax m_intakeRollersMotor =
      new SparkMax(Constants.CanIds.kIntakeRollersMotor, MotorType.kBrushless);
  private final DigitalInput m_deployLowerLimit = new DigitalInput(Constants.DioIds.kIntakeLowerLimit);
  private final DigitalInput m_deployUpperLimit = new DigitalInput(Constants.DioIds.kIntakeUpperLimit);

  /** Creates a new IntakeSub. */
  public IntakeSub() {
    /*
     * SparkMaxConfig configIntakeDeployMotor = new SparkMaxConfig();
     * configIntakeDeployMotor
     * .inverted(true) // Set to true to invert the forward motor direction
     * .smartCurrentLimit(5) // Current limit in amps
     * .idleMode(IdleMode.kBrake) // Set to kCoast to allow the motor to coast when power is 0.0
     * .encoder
     * .positionConversionFactor(Constants.Intake.kDeployEncoderPositionConversionFactor)
     * .velocityConversionFactor(Constants.Intake.kDeployEncoderVelocityConversionFactor); // Set to kCoast to allow the
     * motor to coast when power is 0.0
     * 
     * SparkMaxConfig configIntakeRollersMotor = new SparkMaxConfig();
     * configIntakeRollersMotor
     * .inverted(true) // Set to true to invert the forward motor direction
     * .smartCurrentLimit(5) // Current limit in amps
     * .idleMode(IdleMode.kBrake) // Set to kCoast to allow the motor to coast when power is 0.0
     * .encoder
     * .positionConversionFactor(Constants.Intake.kRollersEncoderPositionConversionFactor)
     * .velocityConversionFactor(Constants.Intake.kRollersEncoderVelocityConversionFactor); // Set to kCoast to allow
     * the motor to coast when power is 0.0
     * 
     * // Save the configuration to the motor
     * // Only persist parameters when configuring the motors on start up as this operation can be slow
     * m_intakeDeployMotor.configure(configIntakeDeployMotor, SparkBase.ResetMode.kResetSafeParameters,
     * SparkBase.PersistMode.kPersistParameters);
     * m_intakeRollersMotor.configure(configIntakeRollersMotor, SparkBase.ResetMode.kResetSafeParameters,
     * SparkBase.PersistMode.kPersistParameters);
     * 
     * // If we change the parameters (e.g. brake mode) during robot operation,
     * // we should not save the changes to flash (i.e. want kNoPersistParameters)
     * //m_intakeMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters,
     * // SparkBase.PersistMode.kNoPersistParameters);
     * init();
     */
  }

  public void init() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Manually set the power of the intake deploy (in/out) motor(s).
   * 
   * @param power power value -1.0 to 1.0
   */
  //public void setDeployPower(double power) {
  //  m_intakeDeployMotor.set(power);
  //}

  /**
   * Manually set the power of the intake rollers motor(s).
   * 
   * @param power power value -1.0 to 1.0
   */
  //public void setRollersPower(double power) {
  //  m_intakeRollersMotor.set(power);
  //}

  /**
   * Sets the current angle as the zero angle
   */
  //public void resetPosition() {
  //  m_intakeDeployMotor.getEncoder().setPosition(0);
  //}

  /**
   * Returns the current angular position of the intake mechanism
   * 
   * @return position in degrees
   */
  //public double getPosition() {
  //  return m_intakeDeployMotor.getEncoder().getPosition();
  //}

  /**
   * Returns the current angular velocity of the intake mechanism
   * 
   * @return velocity in degrees per second
   */
  //public double getVelocity() {
  //  return m_intakeDeployMotor.getEncoder().getVelocity();
  //}

  /**
   * Returns if the intake mechanism is at its lower (all the way inside) limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  //public boolean isAtUpperLimit() {
  //  return m_deployUpperLimit.get(); // If switch is normally closed, return !m_armUpperLimit.get()
  // to return a true when switch is false and false when it's true
  //}

  /**
   * Returns if the intake mechanism is at its upper (all the way outside) limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  //public boolean isAtLowerLimit() {
  //  return m_deployLowerLimit.get(); // If switch is normally closed, return !m_armLowerLimit.get()
  // to return a true when switch is false and false when it's true
  //}

  /**
   * Returns how much current the motor is currently drawing
   * 
   * @return current in amps or -1.0 if motor can't measure current
   */
  //public double getElectricalCurrent() {
  // TODO:  Figure out how to handle returning current for other motors
  //  return m_intakeDeployMotor.getOutputCurrent();
  //}


  //////////////////// Methods used for automated testing ////////////////////
  /**
   * Makes the motor ready for testing. This includes disabling any automation that uses this
   * motor
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  @Override
  public void testEnableMotorTestMode(int motorId) {
    // Disable any mechanism automation (PID, etc.).  Check periodic()
  }

  /**
   * Puts the motor back into normal opeation mode.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  @Override
  public void testDisableMotorTestMode(int motorId) {
    // Re-ensable any mechanism automation
  }

  /**
   * Resets the motor's encoder such that it reads zero
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  @Override
  public void testResetMotorPosition(int motorId) {

  }

  /**
   * Sets the motor's power to the specified value. This needs to also disable anything else from
   * changing the motor power.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   * @param power Desired power -1.0 to 1.0
   */
  @Override
  public void testSetMotorPower(int motorId, double power) {

  }

  /**
   * Returns the motor's current encoder value. Ideally this is the raw value, not the converted
   * value. This should be the INTERNAL encoder to minimize dependencies on other hardware.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   * @return Encoder value in raw or converted units
   */
  @Override
  public double testGetMotorPosition(int motorId) {
    return 0;
  }

  /**
   * Returns the motor's current current-draw.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   * @return Electrical current draw in amps, or -1 if feature not supported
   */
  @Override
  public double testGetMotorAmps(int motorId) {
    return 0;
  }
}
