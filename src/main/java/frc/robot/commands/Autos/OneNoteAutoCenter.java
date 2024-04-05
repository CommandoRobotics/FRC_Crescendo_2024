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
import frc.robot.commands.RevAndShoot;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneNoteAutoCenter extends SequentialCommandGroup {
  /** Creates a new OneNoteAutoCenter. */


  public OneNoteAutoCenter(ArmSubsystem armSubsystem, DispenserSubsystem dispenserSubsystem, AutoAim autoAim, Positioning positioning, SwerveSubsystem swerveSubsystem) {

    //Load all paths
    PathPlannerPath startToCenterNote = PathPlannerPath.fromPathFile("oneNoteAuto");

    addCommands(

      //Reset the robot pose to the starting pose from the first path
      new PrintCommand("Starting OneNoteAutoCenter"),
      Commands.runOnce(() -> swerveSubsystem.resetOdometry(startToCenterNote.getPreviewStartingHolonomicPose())),

      //TODO WE MIGHT NOT NEED TIMEOUTS FOR SETARMSETPOINT (TEST)

      //Move arm up
      new InstantCommand(() -> armSubsystem.setArmSetpoint(90), armSubsystem).repeatedly().withTimeout(1),


      //TODO might be able to replace with LowerArm
                  //Set arm to 55
                  new InstantCommand(() -> armSubsystem.setArmSetpoint(55), armSubsystem).repeatedly().withTimeout(0.5),

                  //Moves arm down to 20
                  new InstantCommand(() -> armSubsystem.setArmSetpoint(20), armSubsystem).repeatedly().withTimeout(0.5),


                  //Moves arm down to 0
                  new InstantCommand(() -> armSubsystem.setArmSetpoint(0), armSubsystem).repeatedly().withTimeout(0.5),

      //Shoot
      new RevAndShoot(dispenserSubsystem),


      //Intake on and follow path
      AutoBuilder.followPath(startToCenterNote)
        .raceWith(dispenserSubsystem.autoIntakeCommand().repeatedly()),


      //Rev up shooter and raise arm
      //Rev Shooter
      new InstantCommand(() -> dispenserSubsystem.spinUpShooterWheels(), dispenserSubsystem),

      //Raise Arm
      //Auto angles the arm //TODO MAY BE ABLE TO CHANGE WITH SET ANGLES
      new AutoAngleArm(33, positioning, autoAim, armSubsystem).repeatedly().withTimeout(1),
                              

      //Shoot
      new InstantCommand(()-> dispenserSubsystem.shootNoteImmediately(), dispenserSubsystem).repeatedly().withTimeout(1)
    );
    
  }


}
