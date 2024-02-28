// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DispenserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {


  // Subsystems
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  ArmSubsystem armSubsystem = new ArmSubsystem();
  DispenserSubsystem dispenserSubsystem = new DispenserSubsystem();

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(0);
  private final CommandXboxController armController =
      new CommandXboxController(1);

  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Default commands
    swerveSubsystem.setDefaultCommand(
      swerveSubsystem.driveCommand(() -> -driverController.getLeftY(),
                                   () -> -driverController.getLeftX(), 
                                   () -> driverController.getRightY()));

    armSubsystem.setDefaultCommand(
        armSubsystem.armCommand(() -> armController.getLeftY()));
        
      
        
    

    

    //swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(() -> (-0.2), () -> (0.2), () -> (0))); // Testing command
  
    // Configure the trigger bindings
    configureBindings();



   

    
//DRIVER STUFF
    //TODO b = autoalign source

    //TODO a = autoalign speaker

    //TODO right trigger = intake

    //TODO left trigger = outtake

    //TODO x = autoalign amp



    //start = reset navx
    driverController.start()
      .onTrue(new InstantCommand(swerveSubsystem::resetGyro, swerveSubsystem));

    

    //ARM STUFF
    //a = spin up shooter 
    armController.a()
      .onTrue(new InstantCommand(dispenserSubsystem::spinUpShooterWheels, dispenserSubsystem))
      .onFalse(new InstantCommand(dispenserSubsystem::stop, dispenserSubsystem));

    //b = intake note
    armController.b()
      .onTrue(new InstantCommand(dispenserSubsystem::ejectNote, dispenserSubsystem))
      .onFalse(new InstantCommand(dispenserSubsystem::stop, dispenserSubsystem));

    //lefttrigger = spinup shooter
    armController.leftTrigger()
      .onTrue(new InstantCommand(dispenserSubsystem::spinUpShooterWheels, dispenserSubsystem))
      .onFalse(new InstantCommand(dispenserSubsystem::stop, dispenserSubsystem));

    //righttrigger =  shoot
    armController.rightTrigger()
      .onTrue(new InstantCommand(dispenserSubsystem::shootNoteImmediately, dispenserSubsystem))
      .onFalse(new InstantCommand(dispenserSubsystem::stop, dispenserSubsystem));

//TODO: make arm up slightly = y

    //TODO: make arm down slightly = a

    //TODO: manual shooter = rightjoystick

    //TODO: arm full up = updpad
    

    //TODO: arm full down = downdpad

    //TODO: arm amp height = right dpad

    //TODO: arm source height = left dpad



    



    
  }

  private void configureBindings() {

    // Driver A: Run the chassis at a set speed forwards
    driverController.a().onTrue(swerveSubsystem.driveCommand(() -> 0.3, () -> 0, () -> 0));
   // armController.a().whileTrue(dispenserSubsystem.driveCommand(() -> 0.3, () -> 0, () -> 0));


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
