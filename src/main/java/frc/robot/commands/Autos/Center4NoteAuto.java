// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Positioning;
import frc.robot.API.AutoAim;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DispenserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.AutoAngleArm;
import frc.robot.commands.LowerArm;
import frc.robot.commands.RevAndShoot;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Center4NoteAuto extends SequentialCommandGroup {
  /** Creates a new OneNoteAutoCenter. */


  public Center4NoteAuto(ArmSubsystem armSubsystem, DispenserSubsystem dispenserSubsystem, AutoAim autoAim, Positioning positioning, SwerveSubsystem swerveSubsystem) {

    //Load all paths
    PathPlannerPath startToCenterNote = PathPlannerPath.fromPathFile("4NStartToCenterNote");
    PathPlannerPath lineToLeftNote = PathPlannerPath.fromPathFile("4NLineToTopNote");
    PathPlannerPath lineToRightNote = PathPlannerPath.fromPathFile("4NLineToBottomNote");


    addCommands(

      new PrintCommand("Starting OneNoteAutoCenter"),

      //Reset the robot pose to the starting pose from the first path
      Commands.runOnce(() -> swerveSubsystem.resetOdometry(startToCenterNote)), // Runs if Red Alliance
      
      //TODO WE MIGHT NOT NEED TIMEOUTS FOR SETARMSETPOINT (TEST)

      //Move arm up
      new InstantCommand(() -> armSubsystem.setArmSetpoint(90), armSubsystem).repeatedly().withTimeout(1),

      //Lower arm
      new LowerArm(armSubsystem),

      //Shoot
      new RevAndShoot(dispenserSubsystem),


      //Intake on and follow path
      AutoBuilder.followPath(startToCenterNote)
        .raceWith(dispenserSubsystem.autoIntakeCommand().repeatedly()),


      //Rev Shooter
      new InstantCommand(() -> dispenserSubsystem.spinUpShooterWheels(), dispenserSubsystem),

      //Auto angles the arm //TODO MAY BE ABLE TO CHANGE WITH SET ANGLES
      new AutoAngleArm(33, positioning, autoAim, armSubsystem).repeatedly().withTimeout(1),
                            

      //Shoot
      new InstantCommand(()-> dispenserSubsystem.shootNoteImmediately(), dispenserSubsystem).repeatedly().withTimeout(1),

      //END OF ONENOTE

      //Lower Arm
      new LowerArm(armSubsystem),

      //Drive to second/left note
      AutoBuilder.followPath(lineToLeftNote)
        .raceWith(dispenserSubsystem.autoIntakeCommand().repeatedly()),

      //Rev Shooter
      new InstantCommand(() -> dispenserSubsystem.spinUpShooterWheels(), dispenserSubsystem),

      //Raise Arm
      //Auto angles the arm //TODO MAY BE ABLE TO CHANGE WITH SET ANGLES
      new AutoAngleArm(33, positioning, autoAim, armSubsystem).repeatedly().withTimeout(1),

      //Shoot
      new InstantCommand(() -> dispenserSubsystem.shootNoteImmediately(), dispenserSubsystem).withTimeout(1),
      dispenserSubsystem.stopCommand(),

      //Lower Arm
      new LowerArm(armSubsystem),

      //Path to third/right note
      AutoBuilder.followPath(lineToRightNote)
        .raceWith(dispenserSubsystem.autoIntakeCommand().repeatedly()),

      //Rev Shooter
      new InstantCommand(() -> dispenserSubsystem.spinUpShooterWheels(), dispenserSubsystem),

      //Raise Arm
      //Auto angles the arm //TODO MAY BE ABLE TO CHANGE WITH SET ANGLES
      new AutoAngleArm(33, positioning, autoAim, armSubsystem).repeatedly().withTimeout(1),

      //Shoot
      new InstantCommand(() -> dispenserSubsystem.shootNoteImmediately(), dispenserSubsystem).withTimeout(1),
      dispenserSubsystem.stopCommand()

      //END OF AUTO
    );
    
  }


}
