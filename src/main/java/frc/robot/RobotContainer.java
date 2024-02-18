// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {


  // Subsystems
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

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
