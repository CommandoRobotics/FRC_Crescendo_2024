// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class LowerArm extends Command {

  ArmSubsystem armSubsystem;
  boolean isFinished = false;
  double startAngle, targetAngle;

  //CHANGE THIS TO CHANGE THE DEFAULT LOWERING SPEED
  double rateOfChange = 1;

  //CHANGE THIS TO CHANGE WHAT ANGLE SHOULD BE CONSIDERED "LOWERED"
  double loweredAngle = 3;

  /** Creates a new LowerArm. */
  public LowerArm(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  /**
   * Lowers the arm to zero </p>
   * 
   * The rate given is the amount of angle that is reduced per scheduler cycle 
   * (typically 20ms)
   * 
   * @param rateOfChange Amount of reduction in the angle in degrees of the arm per cycle 
   */
  public LowerArm(double rateOfChange, ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    this.rateOfChange = rateOfChange;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //targetAngle = armSubsystem.getCurrentArmPosition().getDegrees();
    targetAngle = 90;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Debug comments if needed
    //System.out.println("LowerArm: Current target angle is: " + targetAngle);
    //System.out.println("LowerArm: Current Arm Angle: " + armSubsystem.getCurrentArmPosition().getDegrees());
    armSubsystem.setArmSetpoint(targetAngle);
    targetAngle -= rateOfChange;

    if (armSubsystem.getCurrentArmPosition().getDegrees() <= loweredAngle || targetAngle <= 0) {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
