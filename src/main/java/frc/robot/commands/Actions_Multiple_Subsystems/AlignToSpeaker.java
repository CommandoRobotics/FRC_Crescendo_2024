// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions_Multiple_Subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Positioning;
import frc.robot.API.AutoAim;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToSpeaker extends Command {

  SwerveSubsystem swerveSubsystem;
  ArmSubsystem armSubsystem;
  boolean isFinished = false;
  DoubleSupplier xSpeed, ySpeed;
  double angle;
  AutoAim autoAim;
  Positioning thePositioning;
  double lastGoodDesiredYawInDegrees;
  double desiredAngle;
  double adjustedDesiredAngle;
  double distanceFromSpeaker;

  //TODO Doesn't quite get to the perfect angle
  /** Creates a new AlignToSpeaker. */
  public AlignToSpeaker(double angle, DoubleSupplier xSpeed, DoubleSupplier ySpeed, Positioning thePositioning, AutoAim autoAim, SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.thePositioning = thePositioning;
    this.autoAim = autoAim;
    this.angle = angle;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.armSubsystem = armSubsystem;
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
     distanceFromSpeaker = autoAim.shootingDistanceInMetersToSpeaker(currentPose.getX(), currentPose.getY(), isBlueAlliance);
      lastGoodDesiredYawInDegrees = autoAim.getDesiredYawInDegreesToSpeaker(currentPose.getX(), currentPose.getY(), currentPose.getRotation(), swerveSubsystem.getYaw(),  isBlueAlliance);
       desiredAngle = autoAim.getDesiredShooterAngleInDegrees(currentPose.getX(), currentPose.getY(), isBlueAlliance);
        adjustedDesiredAngle = 59 - desiredAngle; //was 56 orignallu
      // YAGSL uses the X component and Y component of an angle to set the desired angle.
    }
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
       // System.out.println("Desired Yaw:" + lastGoodDesiredYawInDegrees + "  Current (getYaw): " + swerveSubsystem.getYaw().getDegrees() + "  Current(pose): " + swerveSubsystem.getPose().getRotation().getDegrees() + " degrees.");
        //1.358

        
        // armSubsystem.setArmSetpoint(58 - adjustedDesiredAngle); //as of today was 56 //originally 62
         System.out.println("Distance from speaker:" + distanceFromSpeaker);

         if (distanceFromSpeaker < 1.65) {
          armSubsystem.setArmSetpoint(5); //12 at 2
         } 
         else if (distanceFromSpeaker > 1.65 && distanceFromSpeaker < 3.85) {
         armSubsystem.setArmSetpoint(adjustedDesiredAngle); //as of today was 56 //originally 62
         }          
         else if (distanceFromSpeaker > 3.85 && distanceFromSpeaker < 6) {
         armSubsystem.setArmSetpoint(adjustedDesiredAngle + 6); //as of today was 56 //originally 62
         } 

        Rotation2d desiredHeading = Rotation2d.fromDegrees(lastGoodDesiredYawInDegrees);
        double headingX = desiredHeading.getSin();
        double headingY = desiredHeading.getCos();
        swerveSubsystem.drive(xSpeed, ySpeed, () -> headingX, () -> headingY);
        final double toleranceInDegrees = 0.5;
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
