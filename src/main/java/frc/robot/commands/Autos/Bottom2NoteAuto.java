// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Positioning;
import frc.robot.API.AutoAim;
import frc.robot.commands.AutoAngleArm;
import frc.robot.commands.LowerArm;
import frc.robot.commands.RevAndShoot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DispenserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Bottom2NoteAuto extends SequentialCommandGroup {
  /** Creates a new Right2NoteAuto. */
  public Bottom2NoteAuto(ArmSubsystem armSubsystem, DispenserSubsystem dispenserSubsystem, AutoAim autoAim, Positioning positioning, SwerveSubsystem swerveSubsystem) {

    //Load All Paths
    PathPlannerPath startToFarBottomNote = PathPlannerPath.fromPathFile("B2NStartToFarBottomNote");
    PathPlannerPath lineToMidBottomNote = PathPlannerPath.fromPathFile("B2NStartToMidBottomNote");


    addCommands(
      //Reset Robot Pose
      new PrintCommand("Starting bottom 2 note auto"),
      //Reset the robot pose to the starting pose from the first path
      Commands.runOnce(() -> swerveSubsystem.resetOdometry(startToFarBottomNote)), // Runs if Red Alliance

      //Move arm to 90 (to clear bar)
      new InstantCommand(() -> armSubsystem.setArmSetpoint(90), armSubsystem).repeatedly().withTimeout(0.5),

      //Lower Arm
      new LowerArm(armSubsystem),

      //Rev and shoot
      new RevAndShoot(dispenserSubsystem),

      //Drive to far bottom note
      AutoBuilder.followPath(startToFarBottomNote)
        .raceWith(dispenserSubsystem.autoIntakeCommand().repeatedly()),


      //Raise arm and shoot
      new ParallelCommandGroup( new AutoAngleArm(0, positioning, autoAim, armSubsystem),
                                new RevAndShoot(dispenserSubsystem)).withTimeout(3),

      //Drive and intake to mid bottom note
      AutoBuilder.followPath(lineToMidBottomNote)
        .raceWith(dispenserSubsystem.autoIntakeCommand().repeatedly()),

      //Raise arm and shoot
      new ParallelCommandGroup( new AutoAngleArm(0, positioning, autoAim, armSubsystem),
                                new RevAndShoot(dispenserSubsystem))

    );
  }
}
