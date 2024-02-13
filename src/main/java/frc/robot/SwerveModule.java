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

public class SwerveModule implements Sendable,AutoCloseable {
  private static final double kWheelDiameterInMeters = 0.091; // 3.6 inches = 0.091 meters
  private static final double kWheelCircumferanceInMeters = Math.PI * kWheelDiameterInMeters; // 2*Pi*r = Pi*D
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private CANSparkMax m_driveMotor;
  private CANSparkMax m_turningMotor;

  private RelativeEncoder m_driveEncoder;
  private final CANcoder m_turningAbsoluteEncoder;
  // Store these angles using WPILib's Rotation2D class.
  // In some cases we need the angles in degrees, other cases radians, and sometimes rotations.
  // If we stored these angles as double's, we'd have to handle all the conversions, but
  // the Rotation2D object has functions for conversion. It also handles simplifiying multiple
  // rotations into within a single range (i.e. 720 is the same as 0°).
  private final Rotation2d m_turningEncoderOffset; // How far from "forward" our motor is turned when

  // The following are only meant for debugging/display on dashboard. DO NOT USE THEM IN CODE.
  private SwerveModuleState m_debuggingLastDesiredState; // Keeps track of the last requested state for debugging purpoposes.
  private double m_debuggingAccumulatedTestTime; // Used by motor test to keep track of when to change values.

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(1.0, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          .02,
          0.00,
          0.0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.22, 0.0);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(.25, 0.0);

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
      double turningEncoderOffsetInRotations,
      boolean inverted) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningAbsoluteEncoder = new CANcoder(turningEncoderID);
    m_turningEncoderOffset = Rotation2d.fromRotations(turningEncoderOffsetInRotations);
    m_debuggingLastDesiredState = new SwerveModuleState(0.0, new Rotation2d(0)); // Set robot to stopped and straight forward.
    m_debuggingAccumulatedTestTime = 0.0;
    m_driveMotor.setInverted(inverted);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Returns the current state of the module (driving speed and rotation position)
  public SwerveModuleState getState() {
    return new SwerveModuleState(getCurrentSpeedInMetersPerSecond(), getCurrentYaw());
  }

  // Returns the current distance driven by the robot.
  // This is used by Odometry calculations.
  public SwerveModulePosition getPosition() {
    double accumulatedDistanceInMeters = m_driveEncoder.getPosition() * kWheelCircumferanceInMeters; // Total turns of the wheel times wheel circumferance.
    return new SwerveModulePosition(accumulatedDistanceInMeters, getCurrentYaw());
  }

  // Sets the desired speed and angle of the module.
  public void setDesiredState(SwerveModuleState desiredState) {
     var encoderRotation = getCurrentYaw();

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState adjustedDesiredSttate = desiredState;
    //SwerveModuleState adjustedDesiredSttate = SwerveModuleState.optimize(desiredState, encoderRotation);
    m_debuggingLastDesiredState = adjustedDesiredSttate; // Save this off for viewing on the dashboard.

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    adjustedDesiredSttate.speedMetersPerSecond *= adjustedDesiredSttate.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(getCurrentSpeedInMetersPerSecond(), adjustedDesiredSttate.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(adjustedDesiredSttate.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(getCurrentYaw().getRadians(), adjustedDesiredSttate.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }

  public double getCurrentSpeedInMetersPerSecond() {
    double speedInRevolutionsPerMinute = m_driveEncoder.getVelocity();
    double speedInMetersPerMinute = kWheelCircumferanceInMeters * speedInRevolutionsPerMinute;
    double speedinMetersPerSecond = speedInMetersPerMinute / 60.0; // 60 seconds in a minute
    return speedinMetersPerSecond;
  }

  public Rotation2d getCurrentYaw() {
    double rawNumberOfRotations = m_turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
    Rotation2d rawRotation2d = Rotation2d.fromRotations(rawNumberOfRotations);
    Rotation2d correctedRotation = rawRotation2d.minus(m_turningEncoderOffset);
    return correctedRotation;
  }

  public double getMetersDriven() {
    return m_driveEncoder.getPosition() * kWheelCircumferanceInMeters; // Total turns of the wheel times wheel circumferance.
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");
    builder.addDoubleProperty("drivePosition", this::getMetersDriven, null);
    builder.addDoubleProperty("driveCurrentSpeed", this::getCurrentSpeedInMetersPerSecond, null);
    builder.addDoubleProperty("driveTargetSpeed", this::dashboardGetTargetSpeed, null);
    builder.addDoubleProperty("encoderOffsetDegrees", this::dashboardGetEncoderOffsetDegrees, null);
    builder.addDoubleProperty("yawAbsoluteDegrees", this::dasbhoardGetRawYawDegrees, null);
    builder.addDoubleProperty("yawCurrentDegrees", this::dashboardGetCurrentYawDegrees, null);
    builder.addDoubleProperty("yawTargetDegrees", this::dashboardGetDesiredYawDegrees, null);
    builder.addDoubleProperty("yawDeltaDegrees", this::dashboardYawDeltaRawInDegrees, null);
    builder.addDoubleProperty("yawP", this::getYawP, this::setYawP);
    builder.addDoubleProperty("yawI", this::getYawI, this::setYawI);
    builder.addDoubleProperty("yawD", this::getYawD, this::setYawD);
  }

  public double getYawP() {
    return m_turningPIDController.getP();
  }

  public void setYawP(double newP) {
    m_turningPIDController.setP(newP);
  }

  public double getYawI() {
    return m_turningPIDController.getI();
  }

  public void setYawI(double newI) {
    m_turningPIDController.setI(newI);
  }

  public double getYawD() {
    return m_turningPIDController.getD();
  }

  public void setYawD(double newD) {
    m_turningPIDController.setD(newD);
  }

  // The following functions are only meant for the dashbord
  public double dashboardGetCurrentYawDegrees() {
    return getCurrentYaw().getDegrees();
  }

  public double dashboardGetEncoderOffsetDegrees() {
    return m_turningEncoderOffset.getDegrees();
  }

  public double dasbhoardGetRawYawDegrees() {
    return m_turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  public double dashboardGetDesiredYawDegrees() {
    return m_debuggingLastDesiredState.angle.getDegrees();
  }

  // Returns how far (in degreees) we are from the desired value.
  // Values will be between -180 and +180
  public double dashboardYawDeltaRawInDegrees() {
    Rotation2d delta = m_debuggingLastDesiredState.angle.minus(getCurrentYaw());
    return delta.getDegrees();
  }

  public double dashboardGetTargetSpeed() {
    return m_debuggingLastDesiredState.speedMetersPerSecond;
  }

  // Runs a simple test of each motor (drive and turning)
  // Module will drive forward, backwards, stop, rotate Counter-clockwise, and then clockwise.
  public void debuggingRunSimpleMotorTest(double timeSinceLastCall) {
    // update the total time the test has run.
    m_debuggingAccumulatedTestTime += timeSinceLastCall;

    // Set how long each of the tests will run for.
    final double driveForwardEnd = 5;
    final double driveBackwardsEnd = 5 + driveForwardEnd;
    final double turnCounterClockwiseEnd = 0 + driveBackwardsEnd;
    final double turnClockwiseEnd = 0 + turnCounterClockwiseEnd;

    final double driveVoltage = 0.22;
    final double turningVoltage = 0.25;

    // Adjust the timer if we passed the end of the test.
    while (m_debuggingAccumulatedTestTime >= turnClockwiseEnd) {
      m_debuggingAccumulatedTestTime -= turnClockwiseEnd;
    }
    
    if (m_debuggingAccumulatedTestTime < driveForwardEnd) {
      m_driveMotor.setVoltage(driveVoltage);
      m_turningMotor.stopMotor();
    } else if (m_debuggingAccumulatedTestTime < driveBackwardsEnd) {
      m_driveMotor.setVoltage(-driveVoltage);
      m_turningMotor.stopMotor();
    } else if (m_debuggingAccumulatedTestTime < turnCounterClockwiseEnd) {
      m_driveMotor.stopMotor();
      m_turningMotor.setVoltage(turningVoltage);
    } else if (m_debuggingAccumulatedTestTime < turnClockwiseEnd) {
      m_driveMotor.stopMotor();
      m_turningMotor.setVoltage(-turningVoltage);
    } else {
      // Should not make it here
      m_driveMotor.stopMotor();
      m_turningMotor.stopMotor();
    }
  }

  // Interface required for AutoClosable, allowing unit tests to close and open new instances of this class.
  // Anything that you did "new" on in the constructor (or members) should have close (or the appropriate function) called.
  @Override
  public void close() {
    m_driveMotor.close();
    m_turningMotor.close();
    m_turningAbsoluteEncoder.close();
    m_drivePIDController.close();
  }

}
