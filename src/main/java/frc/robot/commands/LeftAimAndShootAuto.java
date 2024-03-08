// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Positioning;
import frc.robot.API.AutoAim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftAimAndShootAuto extends SequentialCommandGroup {
  /** Creates a new AimAndShootCommand. */
  public LeftAimAndShootAuto(Arm armSubsystem, Dispenser dispenserSubsystem, AutoAim autoAim, Positioning positioning, SwerveSubsystem swerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> swerveSubsystem.setGyro(62.06)),
      //62.06                      27.94
        //aims yaw of robot towards speaker
        new AimAtSpeaker(
        armSubsystem, 
        dispenserSubsystem,
        swerveSubsystem,
        autoAim,
        positioning,
        () -> -.3,
        () -> -.3,
        () -> true).repeatedly().withTimeout(5),

        //lowers arm
        new InstantCommand(() -> armSubsystem.manuallyPowerArmRestrained(-.3), armSubsystem).repeatedly().withTimeout(2),

        //revs up shooter
        new InstantCommand(() -> dispenserSubsystem.spinUpShooterWheels(), dispenserSubsystem).repeatedly().withTimeout(1),
        new InstantCommand(() -> dispenserSubsystem.shootNoteImmediately(), dispenserSubsystem).repeatedly().withTimeout(4),

        //stops shooter and puts arm down
        new ParallelCommandGroup(new InstantCommand(() -> dispenserSubsystem.stop(), dispenserSubsystem), 
                                 new InstantCommand(() -> armSubsystem.manuallyPowerArmRestrained(.3), armSubsystem) )
                                 .repeatedly().withTimeout(2),

        //Drives for 1 second
        swerveSubsystem.driveCommand(() -> 0.3,
        ()-> 0,
        ()-> 0).repeatedly().withTimeout(3.5),

        //Stop the drive
        swerveSubsystem.driveCommand(() -> 0,
        ()-> 0,
        ()-> 0).repeatedly().withTimeout(26.5)
    );
  }
}
