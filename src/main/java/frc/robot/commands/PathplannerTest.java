// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
  /** Creates a new PathplannerTest. */
  public PathplannerTest(SwerveSubsystem swerveSubsystem) {
    PathPlannerPath sPath, straigPath;

    sPath = PathPlannerPath.fromPathFile("S forward");
    straigPath = PathPlannerPath.fromPathFile("straight first ring");
    addRequirements(swerveSubsystem);
    addCommands(
      // Reset robot pose to start of first path
      Commands.run(() -> swerveSubsystem.resetOdometry(straigPath.getPreviewStartingHolonomicPose())).withTimeout(1),

      // Follow S Forward
      AutoBuilder.followPath(straigPath)
    );
  }
}
