// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

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

  // Create an instance of our Drivetrain class. That class contains all the ServeModules, etc.
  private final Drivetrain m_swerve = new Drivetrain();

  // This is the function the RoboRio will call repeatedly during autonomous.
  @Override
  public void autonomousPeriodic() {
    //m_swerve.driveMotorTest(getPeriod());
    m_swerve.turnToZero();
    //m_swerve.turnToGyroNorth();

    // Test comment
    
    // Call our authomous program.
    //autonDriveForward();
    // Update where we think we are on the field.
    //m_swerve.updateOdometry();
  }

  // This is the function the RoboRio will call repeatedly during teleop.
  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  // This function simply drives the robot forward (away from our drive station).
  private void autonDriveForward() {
    // Go forward (x) at 50%
    double forwardSpeed = 0.5;
    // No sideways (y) motion (0%)
    double sidewaysSpeed = 0.0;
    // Allow the robot to point whichever direction it is pointing (don't try to rotate it).
    double rotationSpeed = 0.0;
    // Drive "forward", where "forward" means toward the opposite side of the field, not toward the front of the robot.
    // This is "Field-Oriented", instead of Robot-Centric" driving.
    boolean fieldOriented = true;
    // For some of our algorithms to work, we need to pass in the amount of time since our autonomous
    // was last called. The RoboRio calls our autonomous, we update some values, then the Rio goes and does
    // other computer tasks, then it calls our autnomous again, we update values, ...
    // Knowing how much time since we were last called helps us perform accurate guestimates of how close we
    // are to things, or how quickly we are actually moving, etc.
    // Ideally this function is called at a set period, and WPILib provides getPeriod to determine that period.
    double timeSinceLastCall = getPeriod();
    m_swerve.drive(forwardSpeed, sidewaysSpeed, rotationSpeed, fieldOriented, timeSinceLastCall);
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
