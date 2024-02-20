// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;

// The VM is configured to automatically run this class, and to call the functions corresponding to
// each mode, as described in the TimedRobot documentation. If you change the name of this class or
// the package after creating this project, you must also update the manifest file in the resource
// directory.
public class Robot extends TimedRobot {
  private Arm m_arm;
  private Dispenser m_dispenser;
  private final XboxController m_controller = new XboxController(0);

  public Robot() {
    m_dispenser = new Dispenser();
    m_arm = new Arm();
    SmartDashboard.putData("TheArm", m_arm);
  }

  // This function is run when the robot is first started up and should be used for any
  // initialization code.
  @Override
  public void robotInit() {
    m_arm.stop();
    m_dispenser.stop();
  }

  // This function is run once each time the robot enters autonomous mode.
  @Override
  public void autonomousInit() {

  }

  // This function is called periodically during autonomous.
  @Override
  public void autonomousPeriodic() {

  }

  // This function is called once each time the robot enters teleoperated mode.
  @Override
  public void teleopInit() {}

  // This function is called periodically during teleoperated mode.
  @Override
  public void teleopPeriodic() {
    // Arm control
    double armDeadband = 0.5;
    // Y is arm up (Speaker/Source position)
    if (m_controller.getYButton()) {
      m_arm.setAngleInDegrees(90);
      m_arm.autoControl();
    // A is arm Flat (intake postion)
    } else if (m_controller.getAButton()) {
      m_arm.setAngleInDegrees(0);
      m_arm.autoControl();
    // Right joystick (forward/backwards) is manual Arm control
    } else if (m_controller.getRightY() < -armDeadband || armDeadband < m_controller.getRightY()) {
      // The joystick goes from -1.0 to +1.0 and is inverted (forward is negative)
      double armPowerMultiplier = -0.5;
      double armPower = armPowerMultiplier * m_controller.getRightY();
      m_arm.manuallyPowerArm(armPower);
    // If none of the arm controls is pressed, let the arm automatically go to setpoint.
    } else {
      m_arm.autoControl();
    }

    // Dispenser control
    // Left bumper is auto auto intake (until note in indexer)
    if (m_controller.getLeftBumper()) {
      m_dispenser.autoIntake();
    // Right bumper is auto shoot (until empty)
    } else if (m_controller.getRightBumper()) {
      m_dispenser.shootNoteImmediately();
    // X is eject Notes
    } else if (m_controller.getXButtonPressed()) {
      m_dispenser.ejectNote();
    // B is Shoot at full power
    } else if (m_controller.getBButtonPressed()) {
      m_dispenser.shootNoteImmediately();
    // Stop the motors if none of the buttons are pressed.
    } else {
      m_dispenser.stop();
    }
  }

  // This function is called once each time the robot enters test mode.
  @Override
  public void testInit() {}

  // This function is called periodically during test mode.
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationPeriodic() {
    m_arm.simulationPeriodic(getPeriod());
  }
}
