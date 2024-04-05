// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathplannerTest extends SequentialCommandGroup {
  /** Use this to test any one path */
  public PathplannerTest(SwerveSubsystem swerveSubsystem) {
    PathPlannerPath pathToTest;

    pathToTest = PathPlannerPath.fromPathFile("S forward");

    addRequirements(swerveSubsystem);
    addCommands(
      // Reset robot pose to start of first path
      Commands.runOnce(() -> swerveSubsystem.resetOdometry(pathToTest.getPreviewStartingHolonomicPose())),

      // Follow S Forward
      AutoBuilder.followPath(pathToTest)
    );
  }
}
