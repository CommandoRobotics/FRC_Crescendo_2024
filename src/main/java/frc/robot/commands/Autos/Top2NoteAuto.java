// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Positioning;
import frc.robot.API.AutoAim;
import frc.robot.commands.LowerArm;
import frc.robot.commands.RevAndShoot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DispenserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Top2NoteAuto extends SequentialCommandGroup {
  /** Creates a new Right2NoteAuto. */
  public Top2NoteAuto(ArmSubsystem armSubsystem, DispenserSubsystem dispenserSubsystem, AutoAim autoAim, Positioning positioning, SwerveSubsystem swerveSubsystem) {

    //Load All Paths
    PathPlannerPath startToLeftNote = PathPlannerPath.fromPathFile("L2NStartToLeftNote");

    addCommands(
      //Reset Robot Pose
      new PrintCommand("Starting T2NoteAuto"),
      //Reset the robot pose to the starting pose from the first path
      Commands.runOnce(() -> swerveSubsystem.resetOdometry(startToLeftNote)), // Runs if Red Alliance

      //Move arm to 90 (to clear bar)
      new InstantCommand(() -> armSubsystem.setArmSetpoint(90), armSubsystem).repeatedly().withTimeout(1),

      //Lower Arm
      new LowerArm(armSubsystem),

      //Rev and shoot
      new RevAndShoot(dispenserSubsystem),

      //Drive to right note
      AutoBuilder.followPath(startToLeftNote)
        .raceWith(dispenserSubsystem.autoIntakeCommand().repeatedly()),

      //Rev and shoot
      new RevAndShoot(2, dispenserSubsystem),
      dispenserSubsystem.stopCommand()
    );
  }
}
