// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Arm;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private Arm m_arm;
  private final XboxController m_controller = new XboxController(0);
  private double m_accumulatedTestTime;
  private double m_currentSetPointInDegrees;

  public Robot() {
    m_arm = new Arm();
    m_currentSetPointInDegrees = 0.0;
    m_accumulatedTestTime = 0.0;
    SmartDashboard.putData("TheArm", m_arm);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
  }

  // This function is run once each time the robot enters autonomous mode.
  @Override
  public void autonomousInit() {
  }

  // This function is called periodically during autonomous.
  @Override
  public void autonomousPeriodic() {
    m_accumulatedTestTime += getPeriod();
    if (m_accumulatedTestTime > 5.0) {
      // Reset the timer and change the set point.
      m_accumulatedTestTime = 0.0;
      if (m_currentSetPointInDegrees > 45) {
          m_currentSetPointInDegrees = 0;
      } else {
          m_currentSetPointInDegrees = 80;
      }
    }

    m_arm.setAngleInDegrees(m_currentSetPointInDegrees);
    m_arm.autoControl();
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // Read the XBox stick value. Multiply by negative one because XBox controls are inverted (up is negative).
    double xBoxPower = -1.0 * m_controller.getLeftY();
    // Turn off the motors if the controller is close enough to center.
    double stickPower = MathUtil.applyDeadband(xBoxPower, 0.02);
    // Set the arm to this power.
    m_currentSetPointInDegrees += stickPower;
    m_arm.setAngleInDegrees(m_currentSetPointInDegrees);
    m_arm.autoControl();
    //m_arm.manuallyControlArm(stickPower);
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationPeriodic() {
    m_arm.simulationPeriodic(getPeriod());
  }
}
