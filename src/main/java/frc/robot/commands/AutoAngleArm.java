// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Positioning;
import frc.robot.API.AutoAim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAngleArm extends Command {

  Arm armSubsystem;
  boolean isFinished = false;
  double angle;
  AutoAim autoAim;
  Positioning thePositioning;
  double desiredAngle;
  double adjustedDesiredAngle;
  double distanceFromSpeaker;

  //TODO Doesn't quite get to the perfect angle
  /** Creates a new AlignToSpeaker. */
  public AutoAngleArm(double angle, Positioning thePositioning, AutoAim autoAim, Arm armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.thePositioning = thePositioning;
    this.autoAim = autoAim;
    this.angle = angle;
    this.armSubsystem = armSubsystem;
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
       desiredAngle = autoAim.getDesiredShooterAngleInDegrees(currentPose.getX(), currentPose.getY(), isBlueAlliance);
        adjustedDesiredAngle = 56 - desiredAngle;
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
