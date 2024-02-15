// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
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
  private Dispenser m_dispenser;
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();

  public Robot() {
    m_dispenser = new Dispenser();
    m_arm = new Arm();
    SmartDashboard.putData("TheArm", m_arm);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_dispenser.stop();
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Run the intake and shooter at full speed so we can practice our shots.
    m_dispenser.shootNoteImmediately();

  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // Y is shoot.
    if (m_controller.getYButtonPressed()) {
      m_dispenser.shootNoteImmediately();
    // X is intake
    } else if (m_controller.getXButtonPressed()) {
      m_dispenser.intakeNote();
    // B is spin motors (without shooting)
    } else if (m_controller.getBButtonPressed()) {
      m_dispenser.spinUpShooterWheels();
    // Stop the motors if none of the buttons are pressed.
    } else {
      m_dispenser.stop();
    }
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
