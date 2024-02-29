// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dispenser;

import frc.robot.commands.Feeding;

public class RobotContainer {


  // Subsystems
  Arm m_arm = new Arm();
  Dispenser m_dispenser = new Dispenser();
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // Other Utilities
  AutoAim m_autoaim = new AutoAim();
  Positioning m_positioning = new Positioning();

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
      private final CommandXboxController armOperatorController =
      new CommandXboxController(OperatorConstants.kCopilotControllerPort);

  private final JoystickButton armUpButton =
    new JoystickButton(driverController.getHID(), 3);

  

  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

          MathUtil.applyDeadband(driverController.getLeftY(), 0.1);
          MathUtil.applyDeadband(driverController.getLeftX(), 0.1);
          MathUtil.applyDeadband(driverController.getRightX(), 0.1);



    //Default commands
    swerveSubsystem.setDefaultCommand(
      swerveSubsystem.driveCommand(() -> -driverController.getLeftY(),
                                   () -> -driverController.getLeftX(), 
                                   () -> -driverController.getRightX()));
    //swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(() -> (-0.2), () -> (0.2), () -> (0))); // Testing command
    m_dispenser.setDefaultCommand(m_dispenser.spinCommand());
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData("Arm", m_arm);
    SmartDashboard.putData("AutoAim", m_autoaim);
    SmartDashboard.putData("Dispenser", m_dispenser);
    SmartDashboard.putData("Positioning", m_positioning);
  }

  private void configureBindings() {
    // Arm Control
    // Copilot Y: Arm Up (Speaker/Source position)
    armOperatorController.y().whileTrue(Commands.run(() -> m_arm.setAngleInDegrees(90)));
    // Copilot A: Arm Flat (intaking position)
    armOperatorController.a().whileTrue(Commands.run(() -> m_arm.setAngleInDegrees(0)));

    // Dispenser Control
    // Copilot Left Bumper: Auto Intake
    armOperatorController.leftBumper().whileTrue(Commands.run(() -> m_dispenser.autoIntake()));
    // Copilot Right Bumper: Auto Shoot (until empty)
    armOperatorController.rightBumper().whileTrue(Commands.run(() -> m_dispenser.feedShooter()));
    // Copilot X: Eject notes (back through intake)
    armOperatorController.x().whileTrue(Commands.run(() -> m_dispenser.ejectNote()));
    // Copilot B: Shoot (run motors while held)
    armOperatorController.b().whileTrue(Commands.run(() -> m_dispenser.shootNoteImmediately()));
    driverController.start().onTrue(Commands.run(() -> swerveSubsystem.resetGyro()));

    // Chassis
    // Driver 3 (top): Place the arm up in feeding mode (from source or to amp).
   /* armUpButton.whileTrue(
      new Feeding(
        m_arm,
        m_dispenser,
        swerveSubsystem,
        m_autoaim,
        m_positioning,
        () -> driverController.getLeftY(),
        () -> driverController.getLeftX(),
        () -> driverController.trigger().getAsBoolean()
      ) 
    );*/
  }

  public void simulationPeriodic(double period) {
    Pose2d pose = swerveSubsystem.getPose();
    m_positioning.simulationPeriodic(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    double x = 0.5;
    double y = 0.0;
    double headingX = 0.0;
    double headingY = 0.0;
    //double rotateRate = 0.0;
    return swerveSubsystem.driveCommand(
      () -> x,
      () -> y,
      () -> headingX,
      () -> headingY
    );
  }
}
