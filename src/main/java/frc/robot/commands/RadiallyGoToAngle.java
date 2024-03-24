// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class RadiallyGoToAngle extends Command {

  SwerveSubsystem swerveSubsystem;
  boolean isFinished = false;
  DoubleSupplier xSpeed, ySpeed;
  double angle;

  /** Creates a new RadiallyGoToAngle. */
  public RadiallyGoToAngle(double angle, DoubleSupplier xSpeed, DoubleSupplier ySpeed, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.angle = angle;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        double desiredYawInDegrees =  SmartDashboard.getNumber("Yaw Setpoint", 33);
        // YAGSL uses the X component and Y component of an angle to set the desired angle.
        System.out.println("Desired Yaw is " + desiredYawInDegrees + " degrees.");

        Rotation2d desiredHeading = Rotation2d.fromDegrees(desiredYawInDegrees);
        double headingX = desiredHeading.getSin();
        double headingY = desiredHeading.getCos();
        swerveSubsystem.drive(xSpeed, ySpeed, () -> headingX, () -> headingY);
        if (swerveSubsystem.getPose().getRotation().getDegrees() > desiredYawInDegrees - 2 && swerveSubsystem.getPose().getRotation().getDegrees() < desiredYawInDegrees + 2) {
          isFinished = true;
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
