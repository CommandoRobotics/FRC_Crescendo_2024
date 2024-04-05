// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveDisCommand extends Command {

  SwerveSubsystem swerveSubsystem;
  double distance, startingDis, targetDis;
  boolean isForwards = true;  
  boolean isFinished = false;

  /** Creates a new DriveDisCommand. */

  /** @deprecated */
  public DriveDisCommand(double disInInches, SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.distance = disInInches;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startingDis = swerveSubsystem.getDistance();

    if (distance > 0) {
      isForwards = true;
      targetDis = startingDis + Math.abs(distance);
    } else {
      isForwards = false;
      targetDis = startingDis - Math.abs(distance);
    }

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isForwards) {
      if (swerveSubsystem.getDistance() > targetDis) {
        isFinished = true;
      }
    } else {
      if (swerveSubsystem.getDistance() < targetDis) {
        isFinished = true;
      }
    }
    System.out.println("Has not reached target of " + targetDis);
    System.out.println(swerveSubsystem.getDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
