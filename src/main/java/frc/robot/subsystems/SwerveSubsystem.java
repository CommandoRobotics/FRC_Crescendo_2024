// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  //Drivetrain
  SwerveDrive swerveDrive;

  //Constants
  double maximumSpeed = Units.feetToMeters(4.5);

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
   
    //Create the drive by parsing the JSON files in main/deploy/swerve
    try {
      swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      System.out.println("Failed to load the YAGSL swerve directory");
    }

    //Enable HIGH Telemetry
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    SmartDashboard.putData(" field", swerveDrive.field);
    SmartDashboard.putNumber("Yaw Setpoint", 0);

    //configure autobuilder
    AutoBuilder.configureHolonomic(this::getPose, //robot Pose supplier
                                   this::resetOdometry, //method to reset odometry
                                   this::getCurrentSpeeds, //gets the current robot chassisspeeds relative to robot
                                   this::driveRobotRelative, //drives robot robot relative via chassis speeds
                                   new HolonomicPathFollowerConfig(
                                        new PIDConstants(0.0020645, 0.0, 0.0), //translational PID constants
                                        new PIDConstants(0.01, 0.0, 0.0), //Rotational PID constants
                                        4.5, // max module speed m/s
                                        0.4, //drive base radius in meters //TODO fine if this is actually true
                                        new ReplanningConfig() //default pathplanning config
                                   ), 
                                   () -> {
                                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                                    // This will flip the path being followed to the red side of the field.
                                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                      
                                    var alliance = DriverStation.getAlliance();
                                    if (alliance.isPresent()) {
                                      return alliance.get() == DriverStation.Alliance.Red;
                                    }
                                    return false;
                                  },
                                  this // Reference to this subsystem to set requirements
                          );



  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY) 
  {
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getYaw().getRadians(),
                                                                      swerveDrive.getMaximumVelocity()));
    });
  } 

  public void drive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY) 
  {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getYaw().getRadians(),
                                                                      swerveDrive.getMaximumVelocity()));
  } 

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   * 
   * Note: Is relative to the field
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
                        true,
                        false);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @param isFieldOriented  Whether we should be driving field oriented or not
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                              DoubleSupplier angularRotationX, boolean isFieldOriented) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
                        isFieldOriented,
                        false);
    });
  }
    public Command straightTurnPath() {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile("straight 360");
    resetOdometry(path.getPreviewStartingHolonomicPose());

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }

  
    public Command rForward() {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile("R forward");
    resetOdometry(path.getPreviewStartingHolonomicPose());

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }


    
    public Command sForward() {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile("S forward");
    resetOdometry(path.getPreviewStartingHolonomicPose());

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }
  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  public void driveRobotRelative(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
    */


  

  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Returns the distance the swerve has driven in inches
   */
  public double getDistance() {
    return Units.metersToInches(swerveDrive.getModules()[0].getDriveMotor().getPosition());
  }

  //TODO More methods needed to help control the swerve drive.
  // This can be copied from the YAGSL examples or we can make our own ofc

  //gets robots current pose 
  public Pose2d getPose() {
    return swerveDrive.getPose();
  } 

  //resets odometry to given pose //TODO change to 0,0
  public void resetOdometry(Pose2d poseSetpoint) {
    swerveDrive.resetOdometry(poseSetpoint);
  }

  //gets the current chassis speed
  public ChassisSpeeds getCurrentSpeeds(){
   ChassisSpeeds currentHeading = swerveDrive.getRobotVelocity();
   return currentHeading;
  }

  //resets navx to zero
  public Command resetGyroCommand() {
    return run(() -> resetGyro());
  }

  public void resetGyro() {
    swerveDrive.setGyro(new Rotation3d(0, 0, 0));
  }

  //sets gyro to specific setpoint
  public void setGyro(double gyroSetpoint){
    swerveDrive.setGyro(new Rotation3d(0, 0, new Rotation2d(gyroSetpoint).getRadians()));
  }

  //gets the current yaw of the robot
  public Rotation2d getYaw() {
    return Rotation2d.fromRadians(swerveDrive.getGyro().getRotation3d().getZ());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //gets how far the swerve has driven
    SmartDashboard.putNumber("Swerve Distance", swerveDrive.getModules()[0].getDriveMotor().getPosition());

  }


  

}
