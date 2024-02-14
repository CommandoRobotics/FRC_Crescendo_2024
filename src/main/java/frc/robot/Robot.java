// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import java.io.IOException;
import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  //private static Robot   instance;
  //private        Command m_autonomousCommand;

  //private RobotContainer m_robotContainer;
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  "swerve"));

  private double m_accumulatedAutoTime;
  private boolean m_driveForward;
  private final XboxController m_xbox = new XboxController(0);

 //private Timer disabledTimer;

  public Robot()
  {
    //instance = this;
  }

  // public static Robot getInstance()
  // {
  //   return instance;
  // }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // // autonomous chooser on the dashboard.
    // m_robotContainer = new RobotContainer();

    // // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // // immediately when disabled, but then also let it be pushed more 
    // disabledTimer = new Timer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    //CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    // m_robotContainer.setMotorBrake(true);
    // disabledTimer.reset();
    // disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    // if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME))
    // {
    //   m_robotContainer.setMotorBrake(false);
    //   disabledTimer.stop();
    // }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    autoReset();
    // m_robotContainer.setMotorBrake(true);
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null)
    // {
    //   m_autonomousCommand.schedule();
    // }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    autoDriveToPoint();
  }

  public void autoReset() {
    m_driveForward = true;
    m_accumulatedAutoTime = 0.0;
    m_drivebase.resetOdometry(new Pose2d(new Translation2d(0,0), m_drivebase.getHeading()));
  }

  public void autoDriveForward() {
    m_drivebase.drive(new Translation2d(0.5, 0.0), 0.0, false);
  }

  public void autoDriveBackward() {
    m_drivebase.drive(new Translation2d(-0.5, 0.0), 0.0, false);
  }

  public void autoDriveLeft() {
    m_drivebase.drive(new Translation2d(0.0, 0.5), 0.0, false);
  }

  public void autoDriveRight() {
    m_drivebase.drive(new Translation2d(0, -0.5), 0.0, false);
  }

  public void autoFakeDriveForward() {
    m_drivebase.drive(new Translation2d(0.1, 0.0), 0.0, false);
  }

  public void autoFakeDriveLeft() {
    m_drivebase.drive(new Translation2d(0, 0.1), 0.0, false);
  }

  public void autoDriveForwardAndBackwardTimed() {
    m_accumulatedAutoTime += getPeriod();
    if (m_accumulatedAutoTime > 5.0) {
      m_accumulatedAutoTime = 0;
      m_driveForward = !m_driveForward;
    }
    if (m_driveForward) { 
      autoDriveForward();
    } else {
      autoDriveBackward();
    }
  }


  public void autoDriveToPoint() {
    var currentPose = m_drivebase.getPose();
    final double distanceToDrive = 3.0;
    var distanceDriven = currentPose.getX();
    if (distanceDriven <= distanceToDrive) {
      m_drivebase.drive(new Translation2d(.5, 0.0), 0.0, true);
    } else {
      m_drivebase.lock();
    }
  }

  public void autoRelaseMotors() {
    m_drivebase.drive(new Translation2d(0.0, 0.0), 0.0, false);
    m_drivebase.setMotorBrake(false);
  }

  public void autoStopMoving() {
    m_drivebase.drive(new Translation2d(0.0, 0.0), 0.0, false);
    m_drivebase.setMotorBrake(true);
  }

  public void autoRotateClockwise() {
    m_drivebase.drive(new Translation2d(0.0, 0.0), 0.5, false);
  }


  public void autoRotateCounterClockwise() {
    m_drivebase.drive(new Translation2d(0.0, 0.0), 0.5, false);
  }

  public void autoRotateToPoint() {
    var currentPose = m_drivebase.getPose();
    final double degreesToRotate = 90.0;
    var amountRotated = currentPose.getRotation();
    if (amountRotated.getDegrees() <= degreesToRotate) {
      m_drivebase.drive(new Translation2d(0.0, 0.0), 0.5, true);
    } else {
      m_drivebase.lock();
    }
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null)
    // {
    //   m_autonomousCommand.cancel();
    // }
    // m_robotContainer.setDriveMode();
    // m_robotContainer.setMotorBrake(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    if (m_xbox.getXButton()) {
      autoRotateCounterClockwise();
    } else if (m_xbox.getBButton()) {
      autoRotateCounterClockwise();
    } else if (m_xbox.getYButton()) {
      autoDriveForward();
    } else if (m_xbox.getAButton()) {
      autoDriveBackward();
    } else if (m_xbox.getRightBumperPressed()) {
      autoReset();
    } else if (m_xbox.getRightBumper()) {
      autoDriveToPoint();
    } else if (m_xbox.getLeftBumper()) {
      autoRelaseMotors();
    } else if (m_xbox.getLeftTriggerAxis() > 0.1) {
      autoStopMoving();
    } else {
      autoStopMoving();
    }
  }

  @Override
  public void testInit()
  {
    // // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
    // try
    // {
    //   new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    // } catch (IOException e)
    // {
    //   throw new RuntimeException(e);
    // }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}