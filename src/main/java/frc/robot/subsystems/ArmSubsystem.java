// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  CANSparkMax leftarmMotor;
  CANSparkMax rightarmMotor;
  DigitalInput limitSwitch;

  public ArmSubsystem() {
    leftarmMotor = new CANSparkMax(3, MotorType.kBrushless);
    rightarmMotor = new CANSparkMax(4, MotorType.kBrushless);
    DigitalInput limitSwitch = new DigitalInput(3);

  }

  public void setSpeedBotharmMotor(double speed) {
    leftarmMotor.set(speed);
    rightarmMotor.set(speed);
    if (speed > 0 ) {
      if (limitSwitch.get()) {
        motor.set ();
      } else {
        
      }
      }
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
