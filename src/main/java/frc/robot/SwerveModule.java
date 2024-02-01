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

import java.io.IOException;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
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

  private RelativeEncoder m_driveEncoder;
  private final CANcoder m_turningAbsoluteEncoder;
  private final double m_turningEncoderOffsetInRotations;
  private double m_desiredYawInDegrees;
  private double m_accumulatedTestTime; // Used by motor test to keep track of when to change values.

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
      double turningEncoderOffsetInRotations) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningAbsoluteEncoder = new CANcoder(turningEncoderID);
    m_turningEncoderOffsetInRotations = turningEncoderOffsetInRotations;
    //m_turningAbsoluteEncoder.setPosition(m_turningEncoderOffset);
    m_desiredYawInDegrees = 0.0;
    m_accumulatedTestTime = 0.0;

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /// Returns the current state of the module (driving speed and rotation position)
  public SwerveModuleState getState() {
    double speedInRevolutionsPerMinute = m_driveEncoder.getVelocity();
    double speedInMetersPerMinute = kWheelCircumferanceInMeters * speedInRevolutionsPerMinute;
    double speedinMetersPerSecond = speedInMetersPerMinute / 60.0; // 60 seconds in a minute
    return new SwerveModuleState(speedinMetersPerSecond, new Rotation2d(getCurrentYawAdjustedRadians()));
  }

  // Returns the current distance driven by the robot. 
  public SwerveModulePosition getPosition() {
    double accumulatedDistanceInMeters = m_driveEncoder.getPosition() * kWheelCircumferanceInMeters; // Total turns of the wheel times wheel circumferance.
    return new SwerveModulePosition(accumulatedDistanceInMeters, new Rotation2d(getCurrentYawAdjustedRadians()));
  }

  public void runMotorTest(double timeSinceLastCall) {
    m_accumulatedTestTime += timeSinceLastCall;
    final double maxDriveVoltage = 10.0;
    final double maxTurnVoltage = 5.0;
    final double totalTesttime = maxDriveVoltage + maxTurnVoltage;
    if (m_accumulatedTestTime >= totalTesttime) {
      m_accumulatedTestTime = 0.0;
    }
    if (m_accumulatedTestTime < maxDriveVoltage) {
      m_turningMotor.setVoltage(0.0);
      m_driveMotor.setVoltage(m_accumulatedTestTime);
    } else {
      m_driveMotor.setVoltage(0.0);
      m_turningMotor.setVoltage(m_accumulatedTestTime - maxDriveVoltage);
    }
  }

  // Make sure we can steer based on Absolute encoder.
  // FOR TEST ONLY
  public void turnToRawZero() {
    // Report as wanting to go to original zero. Use negative to be opposite the offset.
    double reportedDesiredYawInDegrees = -m_turningEncoderOffsetInRotations * 360.0;
    setDesiredYawInDegrees(reportedDesiredYawInDegrees);

    double rawRotation = m_turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
    // Determine closest direction.
    if (rawRotation > 0.5) {
      rawRotation -= 1.0;
    }
    double motorSpeed = rawRotation;
    if (motorSpeed < 0.05) {
      // Close enough
      m_turningMotor.stopMotor();
    } else {
      m_turningMotor.set(rawRotation);
    }
  }

  // Point wheels toward the front of the robot.
  // FOR TEST ONLY
  public void testTurnToAdjustedZero() {
    testTurnToDirection(0.0);
  }

  // Automatically adjusts motors to point wheel in correct direction.
  // FOR TEST ONLY
  public void testTurnToDirection(double directionInDegrees) {
    // Stop the motor while we are turning during this test.
    m_driveMotor.stopMotor();
    setDesiredYawInDegrees(directionInDegrees);
    adjustTurning();
  }

  public void adjustTurning() {
    // Get the number of degrees to turn. This function should return a value between -180° to +180°.
    double deviationInDegrees = getClosestDeviationInDegrees();
    
    if (deviationInDegrees > -1.0 && deviationInDegrees < 1.0) {
      // Close enough, just stop.
      m_turningMotor.stopMotor();
      m_turningMotor.setIdleMode(IdleMode.kBrake);
    } else {
      // Set the turning speed based on how far away we are (full speed at 180, zero at zero).
      double speed = deviationInDegrees / 180.0;
      if (speed < 0.1) {
        speed = 0.1;
      }
      //m_turningMotor.set(speed);
      m_turningMotor.setVoltage(speed*12.0);
    }
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(getCurrentYawAdjustedRadians());

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
    double rotationInRadians = getCurrentYawAdjustedRotations() * 2 * Math.PI;
    var desiredRadians = state.angle.getRadians();
    m_desiredYawInDegrees = desiredRadians / ( 2 * Math.PI) * 360.0;
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

  // Adjusts the value to within range.
  // For example if you have a value of 370 degrees, but you want it in 0 to 360,
  // this would return 10 degrees (370 - 360 = 10). Or, if you pass in -45 degrees,
  // it would return 315 degrees.
  // Techincally this returns x value that is range_min <= x < rang_max.
  public double normalize(double value, double range_min, double range_max) {
    double total_range = range_max - range_min;
    while (value < range_min) {
      value += total_range;
    }
    while (value >= range_max) {
      value -= total_range;
    }
    return value;
  }

  public double getEncoderOffsetDegrees() {
    return m_turningEncoderOffsetInRotations * 360.0;
  }

  public double getCurrentYawAbsoluteRotations() {
    return m_turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getCurrentYawAbsoluteDegrees() {
    return getCurrentYawAbsoluteRotations() * 360.0;
  }

  public double getCurrentYawAdjustedRotations() {
    return  normalize(getCurrentYawAbsoluteRotations() - m_turningEncoderOffsetInRotations, 0, 1.0);
  }

  public double getCurrentYawAdjustedDegrees() {
    return getCurrentYawAdjustedRotations() * 360.0;
  }

  public double getCurrentYawAdjustedRadians() {
    return getCurrentYawAdjustedRotations() * 2 * Math.PI;
  }

  public double getMetersDriven() {
    return m_driveEncoder.getPosition() * kWheelCircumferanceInMeters; // Total turns of the wheel times wheel circumferance.
  }

  public double getSpeed() {
    return m_driveEncoder.getVelocity();
  }

  public void setDesiredYawInDegrees(double yawInDegrees) {
    m_desiredYawInDegrees = yawInDegrees;
  }

  public double getDesiredYawInRotations() {
    return m_desiredYawInDegrees / 360.0;
  }

  public double getDesiredYawInRadians() {
    return m_desiredYawInDegrees / 360.0 * 2 * Math.PI;
  }

  public double getDesiredYawInDegrees() {
    return m_desiredYawInDegrees;
  }

  // Returns how far (in degreses) we are from the desired value.
  public double getYawDeviationInDegrees() {
    return getDesiredYawInDegrees() - getCurrentYawAdjustedDegrees();
  }

  // Determines the closest direction to turn and how many degrees to turn.
  public double getClosestDeviationInDegrees() {
    return normalize(getYawDeviationInDegrees(), -180, 180);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");
    builder.addDoubleProperty("drivePosition", this::getMetersDriven, null);
    builder.addDoubleProperty("driveSpeed", this::getSpeed, null);
    builder.addDoubleProperty("encoderOffsetDegrees", this::getEncoderOffsetDegrees, null);
    builder.addDoubleProperty("rotationAbsolutePosition", this::getCurrentYawAbsoluteRotations, null);
    builder.addDoubleProperty("rotationAbsoluteDegrees", this::getCurrentYawAbsoluteDegrees, null);
    builder.addDoubleProperty("rotationAdjustedPosition", this::getCurrentYawAdjustedRotations, null);
    builder.addDoubleProperty("rotationAdjustedDegrees", this::getCurrentYawAdjustedDegrees, null);
    builder.addDoubleProperty("rotationAdjustedRadians", this::getCurrentYawAdjustedRadians, null);
    builder.addDoubleProperty("rotationDesiredDegrees", this::getDesiredYawInDegrees, null);
    builder.addDoubleProperty("rotationDesiredRotations", this::getDesiredYawInRotations, null);
    builder.addDoubleProperty("rotationDesiredRadians", this::getDesiredYawInRadians, null);
    builder.addDoubleProperty("rotationDeltaDegrees", this::getYawDeviationInDegrees, null);
    builder.addDoubleProperty("rotationDistanceDegrees", this::getClosestDeviationInDegrees, null);
  }
}
