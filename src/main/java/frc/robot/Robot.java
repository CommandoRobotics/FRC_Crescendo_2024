// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot
 {
  private final Joystick m_joystick = new Joystick(0);
  private final Timer m_timer = new Timer();

  private final AutoAim m_aim = new AutoAim();

  // Used for simulation
  private double m_simulatedX; // Simulated X position (meters forward) on the field.
  private double m_simulatedY; // Simulated Y position (meters left of far right) on the field.
  private Rotation2d m_simulatedYaw; // Simulated Rotation of the robot
  private boolean m_driveForward; // Retains the forward/backwards direction of the robot during simulation.
  private boolean m_driveLeft; // Retains the forward/backwards direction of the robot during simulation.
  // Used by simulation to determine robot position and direction.
  private final Field2d m_field = new Field2d();
  // Used by simulation to pretend to be limelight and write datas over the network that AutoAim will read.
  NetworkTableEntry limelightBotPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose");

  public Robot() {
    m_simulatedX = 5.0;
    m_simulatedY = 5.0;
    m_driveForward = true;
    m_driveLeft = true;
    m_simulatedYaw = new Rotation2d(0);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    super.robotInit();
    
    // Push Field2d to SmartDashboard.
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("AutoAim", m_aim);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Simulates the robot driving around somewhat randomly
    final double robotSpeedInMetersPerSecond = 0.5;
    double distanceTraveled = robotSpeedInMetersPerSecond * getPeriod();

    // Handle driving in X (forward) direction
    // Determine if the robot should turn the other way.
    if (m_simulatedX <= 1.0) {
      // Too close to driver station.
      m_driveForward = true;
    } else if (m_simulatedX >= 10.0) {
      // Driving far from goal.
      m_driveForward = false;
    }
    // Add the amount of distance traveled to our simulated X position.
    if (m_driveForward) {
        m_simulatedX += distanceTraveled;
    } else {
      m_simulatedX -= distanceTraveled;
    }

    // Handle driving in Y (left) direction
    // Determine if the robot should turn the other way.
    if (m_simulatedY <= 0.5) {
      // Too close to driver station.
      m_driveLeft = true;
    } else if (m_simulatedY >= 7.0) {
      // Driving far from goal.
      m_driveLeft = false;
    }
    // Add the amount of distance traveled to our simulated X position.
    if (m_driveLeft) {
        m_simulatedY += distanceTraveled;
    } else {
      m_simulatedY -= distanceTraveled;
    }

    // Send our current information to the network.
    writeFakeLimelightData(m_simulatedX, m_simulatedY, m_simulatedYaw.getDegrees());
    
    // Determine where the AutoAim thinks we should point.
    m_simulatedYaw = Rotation2d.fromRadians(m_aim.getDesiredYawInDegreesToSpeaker(m_simulatedX, m_simulatedY, true));

    // Update the simluation with our new position.
    m_field.setRobotPose(m_simulatedX, m_simulatedY, m_simulatedYaw);
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    final double driveScalar = 0.05;
    // Swap X and Y on XBox Joystick
    m_simulatedX -= m_joystick.getY() * driveScalar;
    m_simulatedY -= m_joystick.getX() * driveScalar;

    // For testing, use button 11 to switch Alliances.
    boolean isBlueAlliance = m_joystick.getRawButton(11);
  
    // Send our current information to the network.
    writeFakeLimelightData(m_simulatedX, m_simulatedY, m_simulatedYaw.getDegrees());
    
    // Determine where the AutoAim thinks we should point.
    // Use button 3 (on top of joystick) to indicate arm up mode (Amp or Source).
    if (m_joystick.getRawButton(3)) {
      m_simulatedYaw = Rotation2d.fromDegrees(m_aim.getDesiredYawInDegreesIfArmUp(m_simulatedX, m_simulatedY, isBlueAlliance));
    } else {
      m_simulatedYaw = Rotation2d.fromDegrees(m_aim.getDesiredYawInDegreesToSpeaker(m_simulatedX, m_simulatedY, isBlueAlliance));
    }

    m_aim.alignedEnoughToShoot(m_simulatedX, m_simulatedY, m_simulatedYaw.getDegrees(), 60, isBlueAlliance);

    // Update the simluation with our new position.
    m_field.setRobotPose(m_simulatedX, m_simulatedY, m_simulatedYaw);
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  private void writeFakeLimelightData(double x, double y, double rotationInDegrees) {
    double[] pose = { m_simulatedX, m_simulatedY, 0, 0, 0, m_simulatedYaw.getDegrees() };
    limelightBotPose.setDoubleArray(pose);
  }

}
