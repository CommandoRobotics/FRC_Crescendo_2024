// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LowerArm;
import frc.robot.commands.RevAndShoot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DispenserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FarBottomNote extends SequentialCommandGroup {
  /** Use this to test any one path */
  public FarBottomNote(SwerveSubsystem swerveSubsystem, DispenserSubsystem dispenserSubsystem, ArmSubsystem armSubsystem) {
    PathPlannerPath farBottomPath;

    farBottomPath = PathPlannerPath.fromPathFile("B2NStartToFarBottomNote");

    addRequirements(swerveSubsystem);
    addCommands(
      // Reset robot pose to start of first path
    Commands.runOnce(() -> swerveSubsystem.resetOdometry(farBottomPath)),

    //Move arm up
    new InstantCommand(() -> armSubsystem.setArmSetpoint(90), armSubsystem).repeatedly().withTimeout(1),

    // lower arm
    new LowerArm(armSubsystem).withTimeout(3),


    //Shoot
    new RevAndShoot(dispenserSubsystem).withTimeout(2),


    //Intake on and follow path
    AutoBuilder.followPath(farBottomPath)
    .raceWith(dispenserSubsystem.autoIntakeCommand().repeatedly()),

    // Rev and shoot
    new RevAndShoot(dispenserSubsystem)

    );
  }
}
