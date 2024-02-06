// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


// FIRST expects this class to exist and we can use it to call our own code.
// During auton, the RoboRio will repeatedly call the autonomousPeriodic() function of this class.
// During teleop, the RoboRio will repeatedly call teleopPeriodic() function of this class.
public class Robot extends TimedRobot {
  // Create our controller (currently an XBox controller)
  private final XboxController m_controller = new XboxController(0);
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // Used by simulation to determine robot position and direction.
  private final Field2d m_field = new Field2d();

  // Create an instance of our Drivetrain class. That class contains all the ServeModules, etc.
  private final Drivetrain m_swerve = new Drivetrain();

  @Override
  public void robotInit() {
    super.robotInit();
    
    // Push Field2d to SmartDashboard.
    SmartDashboard.putData("Field", m_field);
  }

  // This is the function the RoboRio will call repeatedly during autonomous.
  @Override
  public void autonomousPeriodic() {
    //m_swerve.debuggingDriveMotorTest(getPeriod());
    //m_swerve.debuggingDriveInSquare(getPeriod());
    //m_swerve.debuggingDriveToGyroNorth();
    m_swerve.drive(1.0, 0.0, 1.0, true, getPeriod());

    // Update where we think we are on the field.
    m_swerve.updateOdometry();
    m_field.setRobotPose(m_swerve.m_odometry.getPoseMeters());
  }

  // This is the function the RoboRio will call repeatedly during teleop.
  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
    m_field.setRobotPose(m_swerve.m_odometry.getPoseMeters());
  }

  // This function uses joystick movements to control our robot (teleop).
  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
            * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
}
