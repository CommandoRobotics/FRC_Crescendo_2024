// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Positioning;
import frc.robot.API.AutoAim;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToSpeaker extends Command {

  SwerveSubsystem swerveSubsystem;
  boolean isFinished = false;
  DoubleSupplier xSpeed, ySpeed;
  double angle;
  AutoAim autoAim;
  Positioning thePositioning;
  double lastGoodDesiredYawInDegrees;

  /** Creates a new RadiallyGoToAngle. */
  public AlignToSpeaker(double angle, DoubleSupplier xSpeed, DoubleSupplier ySpeed, Positioning thePositioning, AutoAim autoAim, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.thePositioning = thePositioning;
    this.autoAim = autoAim;
    this.angle = angle;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Determine which alliance we are, so we know where our own Speaker is.
    boolean isBlueAlliance = true;
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            isBlueAlliance = false;
        }
    }

    //updates our position
    thePositioning.update();
    if (thePositioning.acquired()) {
      Pose2d currentPose = thePositioning.getPose();
      lastGoodDesiredYawInDegrees = autoAim.getDesiredYawInDegreesToSpeaker(currentPose.getX(), currentPose.getY(), currentPose.getRotation(), swerveSubsystem.getYaw(),  isBlueAlliance);
      // YAGSL uses the X component and Y component of an angle to set the desired angle.
    }
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        System.out.println("Desired Yaw:" + lastGoodDesiredYawInDegrees + "  Current (getYaw): " + swerveSubsystem.getYaw().getDegrees() + "  Current(pose): " + swerveSubsystem.getPose().getRotation().getDegrees() + " degrees.");

        Rotation2d desiredHeading = Rotation2d.fromDegrees(lastGoodDesiredYawInDegrees);
        double headingX = desiredHeading.getSin();
        double headingY = desiredHeading.getCos();
        swerveSubsystem.drive(xSpeed, ySpeed, () -> headingX, () -> headingY);
        final double toleranceInDegrees = 1;
        if (swerveSubsystem.getPose().getRotation().getDegrees() > lastGoodDesiredYawInDegrees - toleranceInDegrees && swerveSubsystem.getPose().getRotation().getDegrees() < lastGoodDesiredYawInDegrees + toleranceInDegrees) {
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
