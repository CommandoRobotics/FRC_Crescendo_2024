// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Positioning;
import frc.robot.API.AutoAim;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DispenserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.AutoAngleArm;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneNoteAutoCenter extends SequentialCommandGroup {
  /** Creates a new OneNoteAutoCenter. */

  PathPlannerPath path1 = PathPlannerPath.fromPathFile("straight first ring");



  public OneNoteAutoCenter(ArmSubsystem armSubsystem, DispenserSubsystem dispenserSubsystem, AutoAim autoAim, Positioning positioning, SwerveSubsystem swerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    //moves arm up
    new InstantCommand(() -> armSubsystem.setArmSetpoint(90), armSubsystem).repeatedly().withTimeout(1),



    //moves arm down to 55
    new InstantCommand(() -> armSubsystem.setArmSetpoint(55), armSubsystem).repeatedly().withTimeout(0.5),

    //moves arm down to 20
    new InstantCommand(() -> armSubsystem.setArmSetpoint(20), armSubsystem).repeatedly().withTimeout(0.5),


    //moves arm down to 0
    new InstantCommand(() -> armSubsystem.setArmSetpoint(0), armSubsystem).repeatedly().withTimeout(0.5),


    //revs up 
    new InstantCommand(()-> dispenserSubsystem.spinUpShooterWheels(), dispenserSubsystem).repeatedly().withTimeout(2.5),


    //shoot
    new InstantCommand(() -> dispenserSubsystem.shootNoteImmediately(), dispenserSubsystem).repeatedly().withTimeout(1),


    //intake on and follow path
    new ParallelCommandGroup(dispenserSubsystem.autoIntakeCommand().repeatedly(),
                            swerveSubsystem.oneNoteCenter()).withTimeout(4),


    //rev up shooter and raise arm
    new ParallelCommandGroup(new AutoAngleArm(33,
    positioning,
    autoAim,
    armSubsystem).repeatedly(),
    new InstantCommand(()-> dispenserSubsystem.spinUpShooterWheels(), dispenserSubsystem)
    ).repeatedly().withTimeout(2.5),
                            

    //shoot
     new InstantCommand(()-> dispenserSubsystem.shootNoteImmediately(), dispenserSubsystem).repeatedly().withTimeout(2.5)

    
    );
    
  }


}
