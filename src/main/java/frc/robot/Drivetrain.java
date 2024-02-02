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
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

  private final SwerveModule m_frontLeft = new SwerveModule(2, 3, 11, 0.266113 );
  private final SwerveModule m_frontRight = new SwerveModule(4, 5, 12, 0.962891 );
  private final SwerveModule m_backLeft = new SwerveModule(6, 7, 13, 0.225098 );
  private final SwerveModule m_backRight = new SwerveModule(8, 9, 14, 0.232910);

  private final AHRS m_navXMXP = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private double m_debuggingAccumulatedTestTime;

  public final SwerveDriveOdometry m_odometry =
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
    m_debuggingAccumulatedTestTime = 0.0;
  }

  // Returns robot angle is -180° to +180°
  private Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(m_navXMXP.getYaw());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    // Determine our the speeds (forward, sideways, and rotation) our chassis should move.
    ChassisSpeeds speedToDrive = new ChassisSpeeds(xSpeed, ySpeed, rot);
    // If we want to be Field-Oriented driving, modify these based on the direction robot is actually pointing.
    if (fieldRelative) {
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroYaw());
    }

    // Get the speed/rotation each individual Swerve Module needs to drive at.
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speedToDrive, periodSeconds));

    // Slow down any excessive speeds so it does not try to drive faster than it is capable of.
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    // Control each swerve module using the unique speeds calculated above.
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  // Use the WPILib algorithms to guess where we are/will be.
  // This takes in the current state of each Swerve module, and uses Dead Reckoning.
  // This means it guesses where we are based on how fast our motors are going, and
  // how long since the last time we updated that information.
  // For example, if all our motors were pointing North and at 1 foot per second,
  // and we last called this one half second ago, the algorithm would assume our
  // point on the map moved half a foot North (i.e. 1ft/s x 0.5s = 0.5ft).
  public void updateOdometry() {
    
    m_odometry.update(
        getGyroYaw(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  //
  //
  // KEEP ITEMS USED FOR CONTROLLING DRIVETRAIN ABOVE HERE
  // KEEP ITEMS ONLY USED FOR DEBUGGING (including dashboard) BELOW HERE
  //
  //

  // This function allows the SmartDashboard to see our class as "Sendable".
  // The values in this function will be regularly updated by the SmartDashboard so we can view them while driving/testing.
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("TheDrivetrain");
    builder.addDoubleProperty("gyroYawDegrees", this::dashboardGyroYawDegrees, null);
  }

  private double dashboardGyroYawDegrees() {
    return getGyroYaw().getDegrees();
  }

  public void debuggingDriveMotorTest(double periodSeconds) {
    m_frontLeft.debuggingRunSimpleMotorTest(periodSeconds);
    m_frontRight.debuggingRunSimpleMotorTest(periodSeconds);
    m_backLeft.debuggingRunSimpleMotorTest(periodSeconds);
    m_backRight.debuggingRunSimpleMotorTest(periodSeconds);
  }

  public void debuggingTurnToAdjustedZero() {
    SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d(0.0));
    debuggingDriveAllSameState(desiredState);
  }

  public void debuggingTurnToGyroNorth() {
    SwerveModuleState desiredState = new SwerveModuleState(0.0, getGyroYaw());
    debuggingDriveAllSameState(desiredState);
  }

  public void debuggingDriveToGyroNorth() {
    double desiredSpeedInMetersPerSecond = 0.5;
    SwerveModuleState desiredState = new SwerveModuleState(desiredSpeedInMetersPerSecond, getGyroYaw());
    debuggingDriveAllSameState(desiredState);
  }

  public void debuggingDriveAllSameState(SwerveModuleState desiredState) {
    m_frontLeft.setDesiredState(desiredState);
    m_frontRight.setDesiredState(desiredState);
    m_backLeft.setDesiredState(desiredState);
    m_backRight.setDesiredState(desiredState);
  }

  public void debuggingDriveInSquare(double periodSeconds) {
    final double driveSideInSeconds = 3.0;
    final double turningPauseInSeconds = 2.0;
    // Determine the time it takes to get to each point (used later)
    final double firstLegComplete = driveSideInSeconds;
    final double firstTurnComplete = firstLegComplete + turningPauseInSeconds;
    final double secondLegComplete = firstTurnComplete + driveSideInSeconds;
    final double secondTurnComplete = secondLegComplete + turningPauseInSeconds;
    final double thirdLegComplete = secondTurnComplete + driveSideInSeconds;
    final double thirdTurnComplete = thirdLegComplete + turningPauseInSeconds;
    final double fourthLegComplete = thirdTurnComplete + driveSideInSeconds;
    final double fourthTurnComplete = fourthLegComplete + turningPauseInSeconds;
    final double driveSpeedInMetersPerSecond = 0.5;
    final double quarterRotationInRadians = 2 * Math.PI / 4;
    // If the timer has gone longer than one test, reset the timer.
    m_debuggingAccumulatedTestTime += periodSeconds;
    while (m_debuggingAccumulatedTestTime > fourthTurnComplete) {
      m_debuggingAccumulatedTestTime -= fourthTurnComplete;
    }
    if (m_debuggingAccumulatedTestTime < firstLegComplete) {
      // Drive forward
      debuggingDriveAllSameState(new SwerveModuleState(driveSpeedInMetersPerSecond, new Rotation2d(0 * quarterRotationInRadians)));
    } else if (m_debuggingAccumulatedTestTime < firstTurnComplete) {
      // Rotate wheels left
      debuggingDriveAllSameState(new SwerveModuleState(0.0, new Rotation2d(1 * quarterRotationInRadians)));
    } else if (m_debuggingAccumulatedTestTime < secondLegComplete) {
      // Drive left
      debuggingDriveAllSameState(new SwerveModuleState(driveSpeedInMetersPerSecond, new Rotation2d(1 * quarterRotationInRadians)));
    } else if (m_debuggingAccumulatedTestTime < secondTurnComplete) {
      // Rotate wheels backwards
      debuggingDriveAllSameState(new SwerveModuleState(0.0, new Rotation2d(2 * quarterRotationInRadians)));
    } else if (m_debuggingAccumulatedTestTime < thirdLegComplete) {
      // Drive backwards
      debuggingDriveAllSameState(new SwerveModuleState(driveSpeedInMetersPerSecond, new Rotation2d(2 * quarterRotationInRadians)));
    } else if (m_debuggingAccumulatedTestTime < thirdTurnComplete) {
      // Rotate wheels right
      debuggingDriveAllSameState(new SwerveModuleState(0.0, new Rotation2d(3 * quarterRotationInRadians)));
    } else if (m_debuggingAccumulatedTestTime < fourthLegComplete) {
      // Drive right
      debuggingDriveAllSameState(new SwerveModuleState(driveSpeedInMetersPerSecond, new Rotation2d(2 * quarterRotationInRadians)));
    } else if (m_debuggingAccumulatedTestTime < fourthTurnComplete) {
      // Rotate wheels forward
      debuggingDriveAllSameState(new SwerveModuleState(0.0, new Rotation2d(0 * quarterRotationInRadians)));
    } else {
      // Hmm... shouldn't have made it here. Just stop.
      debuggingDriveAllSameState(new SwerveModuleState(0.0, new Rotation2d(0.0)));
    }
  }

}