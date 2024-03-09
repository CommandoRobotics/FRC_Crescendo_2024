// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Positioning;
import frc.robot.API.AutoAim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightAimAndShootAuto extends SequentialCommandGroup {
  /** Creates a new AimAndShootCommand. */
  public RightAimAndShootAuto(Arm armSubsystem, Dispenser dispenserSubsystem, AutoAim autoAim, Positioning positioning, SwerveSubsystem swerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //TODO make robot not field centric in auto
        //lowers arm
       // new InstantCommand(() -> armSubsystem.setArmSetpoint(Constants.ArmConstants.kSubwooferAngle), armSubsystem).repeatedly().withTimeout(4),

        
        //revs up shooter and raises arm
        new ParallelCommandGroup(new InstantCommand(() -> armSubsystem.setArmSetpoint(25), armSubsystem).repeatedly().withTimeout(2),
                                 new InstantCommand(() -> dispenserSubsystem.spinUpShooterWheels(), dispenserSubsystem).repeatedly().withTimeout(1)
                                 ),

        //shoots
        new InstantCommand(() -> dispenserSubsystem.shootNoteImmediately(), dispenserSubsystem).repeatedly().withTimeout(4),
       
        //stops shooter and puts arm down
        new ParallelCommandGroup(new InstantCommand(() -> dispenserSubsystem.stop(), dispenserSubsystem), 
                                 new InstantCommand(() -> armSubsystem.setArmSetpoint(0), armSubsystem) )
                                 .repeatedly().withTimeout(3),
        


        // drives backwards a little bit //TODO find actual distance
        new DriveDisCommand(-17, swerveSubsystem)
        .raceWith(swerveSubsystem.driveCommand(() -> 0.3, ()-> 0, ()-> 0, false).repeatedly()),
        
        //Stop the drive
        swerveSubsystem.driveCommand(
          () -> 0,
          ()-> 0,
          ()-> 0).repeatedly().withTimeout(1),


        //turns sin, cosc to 90 degrees //TODO get values
       swerveSubsystem.driveCommand(()-> 0, ()-> 0, () -> 0, () -> 1).withTimeout(1.5),

      
        //Stop the drive
        swerveSubsystem.driveCommand(
          () -> 0,
          ()-> 0,
          ()-> 0).repeatedly().withTimeout(1),





        
        // drives backwards and intakes //TODO find actual distance
        new DriveDisCommand(-60, swerveSubsystem).alongWith(new InstantCommand(() -> dispenserSubsystem.autoIntake(),dispenserSubsystem ))
        .raceWith(swerveSubsystem.driveCommand(() -> 0.3, ()-> 0, ()-> 0, false).repeatedly()),
        
        //Stop the drive
        swerveSubsystem.driveCommand(
          () -> 0,
          ()-> 0,
          ()-> 0).repeatedly().withTimeout(1),

        //turn
        //turns sin, cosc at -62.07 //TODO get values
       swerveSubsystem.driveCommand(()-> 0, ()-> 0, () -> Math.sin(-62.07), () -> Math.cos(62.07)).withTimeout(2.5),


        //Stops intake and Drives Forwards //TODO find distance
        new ParallelCommandGroup(new InstantCommand(() -> dispenserSubsystem.stop(), dispenserSubsystem).repeatedly().withTimeout(1),
                                 new DriveDisCommand(60, swerveSubsystem)
                                  .raceWith(swerveSubsystem.driveCommand(() ->- 0.3, ()-> 0, ()-> 0, false).repeatedly())),
      

        //Stop the drive
        swerveSubsystem.driveCommand(
          () -> 0,
          ()-> 0,
          ()-> 0).repeatedly().withTimeout(1),

        //revs up shooter and raises arm
        new ParallelCommandGroup(new InstantCommand(() -> armSubsystem.setArmSetpoint(25), armSubsystem).repeatedly().withTimeout(2),
                                 new InstantCommand(() -> dispenserSubsystem.spinUpShooterWheels(), dispenserSubsystem).repeatedly().withTimeout(2)),

        //shoots
        new InstantCommand(() -> dispenserSubsystem.shootNoteImmediately(), dispenserSubsystem).repeatedly().withTimeout(1.5),

        //stops 
        new ParallelCommandGroup(new InstantCommand(() -> armSubsystem.stop(), armSubsystem).repeatedly().withTimeout(2),
                                 new InstantCommand(() -> dispenserSubsystem.stop(), dispenserSubsystem).repeatedly().withTimeout(2))
    );
  }
}
