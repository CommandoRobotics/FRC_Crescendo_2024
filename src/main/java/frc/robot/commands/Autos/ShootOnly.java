// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Positioning;
import frc.robot.API.AutoAim;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DispenserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.AutoAngleArm;
import frc.robot.commands.LowerArm;
import frc.robot.commands.RevAndShoot;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootOnly extends SequentialCommandGroup {
  /** Creates a new OneNoteAutoCenter. */


  public ShootOnly(ArmSubsystem armSubsystem, DispenserSubsystem dispenserSubsystem, AutoAim autoAim, Positioning positioning, SwerveSubsystem swerveSubsystem) {

    //Load all paths
    PathPlannerPath driveTaxi = PathPlannerPath.fromPathFile("DriveTaxi");

    addCommands(

      new PrintCommand("Starting OneNoteAutoCenter"),

      //Reset the robot pose to the starting pose from the first path
      Commands.runOnce(() -> swerveSubsystem.resetOdometry(driveTaxi)), // Runs if Red Alliance
      
      //TODO WE MIGHT NOT NEED TIMEOUTS FOR SETARMSETPOINT (TEST)

      //Move arm up
      new InstantCommand(() -> armSubsystem.setArmSetpoint(90), armSubsystem).repeatedly().withTimeout(1),

      // Move arm down
      new LowerArm(armSubsystem),



      new RevAndShoot(dispenserSubsystem).withTimeout(2),

      new WaitCommand(9),

       AutoBuilder.followPath(driveTaxi)
    );
    
  }


}
