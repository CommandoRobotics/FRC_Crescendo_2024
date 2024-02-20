// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dispenser;

public class RobotContainer {


  // Subsystems
  Arm m_arm = new Arm();
  Dispenser m_dispenser = new Dispenser();
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
      private final CommandXboxController armOperatorController =
      new CommandXboxController(OperatorConstants.kCopilotControllerPort);

  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Default commands
    swerveSubsystem.setDefaultCommand(
      swerveSubsystem.driveCommand(() -> -driverController.getLeftY(),
                                   () -> -driverController.getLeftX(), 
                                   () -> driverController.getRightY()));
    //swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(() -> (-0.2), () -> (0.2), () -> (0))); // Testing command
  
    // Configure the trigger bindings
    configureBindings();
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
    armOperatorController.leftBumper().whileTrue(Commands.run(() -> m_dispenser.shootNoteImmediately()));
    // Copilot X: Eject notes (back through intake)
    armOperatorController.x().whileTrue(Commands.run(() -> m_dispenser.ejectNote()));
    // Copilot B: Shoot (run motors while held)
    armOperatorController.x().whileTrue(Commands.run(() -> m_dispenser.shootNoteImmediately()));

    // Chassis
    // Driver A: Run the chassis at a set speed forwards
    driverController.a().onTrue(swerveSubsystem.driveCommand(() -> 0.3, () -> 0, () -> 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null; //TODO REPLACE THIS
  }
}
