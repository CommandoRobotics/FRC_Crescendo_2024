// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
  double intakeSpeed = 0.0;
  
  public CANSparkMax intakeSpark = new CANSparkMax(1, MotorType.kBrushless);
  public RelativeEncoder intakeEncoder = intakeSpark.getEncoder();
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}
  
  //sets intake in
  public void intakeIn(){
    intakeSpark.set(Constants.kIntakeSpeed);
    }

  //sets intake out
  public void intakeOut(){
    intakeSpark.set(-1);
    }

  //stops intake
  public void stop(){
    intakeSpark.stopMotor();
  }

  //sets intake speed
  public void setSpeed(double intakeSpeed){
     
    intakeSpark.set(intakeSpeed);

  }


  //sets intake speed
  public double getSpeed(){
     
    double velocity = intakeEncoder.getVelocity();
    return velocity;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
