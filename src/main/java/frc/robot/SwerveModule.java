// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModule implements Sendable {
  private static final double kWheelDiameterInMeters = 0.091; // 3.6 inches = 0.091 meters
  private static final double kWheelCircumferanceInMeters = Math.PI * kWheelDiameterInMeters; // 2*Pi*r = Pi*D
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private CANSparkMax m_driveMotor;
  private CANSparkMax m_turningMotor;

  //private final Encoder m_driveEncoder;
  //private final Encoder m_turningEncoder;
  private RelativeEncoder m_driveEncoder;
  private final CANcoder m_turningAbsoluteEncoder;
  private final double m_turningEncoderOffset;
  private double m_desiredRadians;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(.5, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          .5,
          0.1,
          0.2,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param turningEncoderID CAN ID of turning encoder.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderID,
      double turningEncoderOffset) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningAbsoluteEncoder = new CANcoder(turningEncoderID);
    m_turningEncoderOffset = turningEncoderOffset;
    m_turningAbsoluteEncoder.setPosition(m_turningEncoderOffset);
    m_desiredRadians = 0.0;

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double speedInRevolutionsPerMinute = m_driveEncoder.getVelocity();
    double speedInMetersPerMinute = kWheelCircumferanceInMeters * speedInRevolutionsPerMinute;
    double speedinMetersPerSecond = speedInMetersPerMinute / 60.0; // 60 seconds in a minute
    return new SwerveModuleState(speedinMetersPerSecond, new Rotation2d(getTurningAdjustedPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    double accumulatedDistanceInMeters = m_driveEncoder.getPosition() * kWheelCircumferanceInMeters; // Total turns of the wheel times wheel circumferance.
    return new SwerveModulePosition(accumulatedDistanceInMeters, new Rotation2d(getTurningAdjustedPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    double currentSpeedInRevolutionsPerMinute = m_driveEncoder.getVelocity();
    double currentSpeedInMetersPerMinute = kWheelCircumferanceInMeters * currentSpeedInRevolutionsPerMinute;
    double currentSpeedInMetersPerSecond = currentSpeedInMetersPerMinute / 60.0; // 60 seconds in a minute

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = 
    m_drivePIDController.calculate(currentSpeedInMetersPerSecond, state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    double rotationInRadians = getTurningAdjustedPosition() * 2 * Math.PI;
    m_desiredRadians = state.angle.getRadians();
    final double turnOutput = m_turningPIDController.calculate(rotationInRadians, state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(0);
    //m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    double totalTurnVoltage = turnOutput;
    if (totalTurnVoltage < 0.5) {
      totalTurnVoltage = 0;
    }
    m_turningMotor.setVoltage(totalTurnVoltage);
  }

  public double getTurningAbsoluteRotation() {
    return m_turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getTurningAdjustedPosition() {
    return m_turningEncoderOffset + getTurningAbsoluteRotation();
  }

  public double getTurningAdjustedRadians() {
    return getTurningAdjustedPosition() * 2 * Math.PI;
  }

  public double getMetersDriven() {
    return m_driveEncoder.getPosition() * kWheelCircumferanceInMeters; // Total turns of the wheel times wheel circumferance.
  }

  public double getSpeed() {
    return m_driveEncoder.getVelocity();
  }

  public double getDesiredRadians() {
    return m_desiredRadians;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");
    builder.addDoubleProperty("drivePosition", this::getMetersDriven, null);
    builder.addDoubleProperty("driveSpeed", this::getSpeed, null);
    builder.addDoubleProperty("rotationAbsolutePosition", this::getTurningAbsoluteRotation, null);
    builder.addDoubleProperty("rotationAdjustedRadians", this::getTurningAdjustedRadians, null);
    builder.addDoubleProperty("rotationDesiredRadians", this::getDesiredRadians, null);
    
  }
}
