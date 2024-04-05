// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DispenserSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RevAndShoot extends SequentialCommandGroup {
  
  /** Revs the dispenser and shoots a note (wrapper command) */
  public RevAndShoot(DispenserSubsystem dispenserSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Revs up 
      new InstantCommand(()-> dispenserSubsystem.spinUpShooterWheels(), dispenserSubsystem).repeatedly().withTimeout(1.5),

      //Shoot
      new InstantCommand(() -> dispenserSubsystem.shootNoteImmediately(), dispenserSubsystem).repeatedly().withTimeout(1)

    );
  }

  /**
   * Rev the shooter and shoot a note (wrapper command)
   * @param revTimeSec Allows you to change the rev length in seconds
   * @param dispenserSubsystem
   */
  public RevAndShoot(double revTimeSec, DispenserSubsystem dispenserSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Revs up 
      new InstantCommand(()-> dispenserSubsystem.spinUpShooterWheels(), dispenserSubsystem).repeatedly().withTimeout(revTimeSec),

      //Shoot
      new InstantCommand(() -> dispenserSubsystem.shootNoteImmediately(), dispenserSubsystem).repeatedly().withTimeout(1)

    );
  }
}
