// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;


/** Represents a swerve drive style drivetrain. */
public class Drivetrain implements Sendable {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.311, 0.311);
  private final Translation2d m_frontRightLocation = new Translation2d(0.311, -0.311);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.311, 0.311);
  private final Translation2d m_backRightLocation = new Translation2d(-0.311, -0.311);

  private final SwerveModule m_frontLeft = new SwerveModule(2, 3, 11, 0.268);
  private final SwerveModule m_frontRight = new SwerveModule(4, 5, 12, 0.461);
  private final SwerveModule m_backLeft = new SwerveModule(6, 7, 13, 0.226);
  private final SwerveModule m_backRight = new SwerveModule(8, 9, 14, -0.254);

  private final AHRS m_navXMXP = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          new Rotation2d(m_navXMXP.getAngle()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_navXMXP.reset();
    SmartDashboard.putData("FLSwerve", m_frontLeft);
    SmartDashboard.putData("FRSwerve", m_frontRight);
    SmartDashboard.putData("BLSwerve", m_backLeft);
    SmartDashboard.putData("BRSwerve", m_backRight);
    SmartDashboard.putData("MyDrivetrain", this);
  }

  private double getGyroAngle() {
    return m_navXMXP.getAngle();
  }

  private Rotation2d getGyroRotation2d() {
    return new Rotation2d(getGyroAngle());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, getGyroRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        getGyroRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

   @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("TheDrivetrain");
    builder.addDoubleProperty("CurrentAngle", this::getGyroAngle, null);
  }

}