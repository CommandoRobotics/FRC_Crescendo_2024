// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.API.AutoAim;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DispenserSubsystem;
import frc.robot.commands.*;
import frc.robot.commands.Actions_Multiple_Subsystems.*;
import frc.robot.commands.Autos.*;
import frc.robot.commands.Swerve.*;;

public class RobotContainer {

  // Subsystems
  ArmSubsystem armSubsystem = new ArmSubsystem();
  DispenserSubsystem dispenserSubsystem = new DispenserSubsystem();
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // Other Utilities
  AutoAim m_autoaim = new AutoAim();
  Positioning m_positioning = new Positioning();

  // Creates autochooser
  SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // LED Pin
  DigitalOutput ledPin = new DigitalOutput(5);
  Alliance prevAlliance = Alliance.Blue;
  


  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Start the driver camera
    CameraServer.startAutomaticCapture();

    // Pathplanner command registrations //TODO add these 

    // Register Named Commands for PathPlanner
    // NamedCommands.registerCommand("auto aim", new AlignToSpeaker(33,
    //                                                       () -> -driverController.getLeftY(),
    //                                                       () -> -driverController.getLeftX(),
    //                                                       m_positioning,
    //                                                       m_autoaim,
    //                                                       swerveSubsystem,
    //                                                       armSubsystem
    //                                                       ));
    // NamedCommands.registerCommand("shoot", dispenserSubsystem.forceShootCommand());
    // NamedCommands.registerCommand("auto intake", dispenserSubsystem.autoIntakeCommand());
    // NamedCommands.registerCommand("arm down", Commands.run(() -> armSubsystem.setArmSetpoint(0), armSubsystem));  
    // NamedCommands.registerCommand("spin shooter", dispenserSubsystem.spinCommand());  
    // NamedCommands.registerCommand("stop shooter", dispenserSubsystem.stopCommand()); 
    // NamedCommands.registerCommand("stop arm", armSubsystem.stopCommand());  
    // NamedCommands.registerCommand("auto arm", new AutoAngleArm(33, m_positioning, m_autoaim, armSubsystem));  
    // NamedCommands.registerCommand("arm up", Commands.run(() -> armSubsystem.setArmSetpoint(90), armSubsystem));  


      
    // Default Commands

    // Swerve Subsystem: Default drive using the driver controller (field oriented)
    swerveSubsystem.setDefaultCommand(
      swerveSubsystem.driveCommand(() -> -MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
                                  () -> -MathUtil.applyDeadband(driverController.getLeftX(), 0.1), 
                                  () -> -MathUtil.applyDeadband(driverController.getRightX(), 0.1),
                                  true));

  
    // ArmSubystem: Manual control of the arm - operator left stick Y
    armSubsystem.setDefaultCommand(
      new InstantCommand(
        () -> armSubsystem.manuallyPowerArmRestrained(MathUtil.applyDeadband(-operatorController.getLeftY(), 0.1)),
        armSubsystem).repeatedly());

    // DispenserSubsystem: Manual control of the shooter
    dispenserSubsystem.setDefaultCommand(
      dispenserSubsystem.manualShootCommand(
        () -> MathUtil.applyDeadband(-operatorController.getRightY(), 0.1)).repeatedly());

    //TODO add LED default command

    // Configure the trigger bindings
    configureBindings();

    // Adds data from different subsystems onto smart dashboard
    SmartDashboard.putData(armSubsystem);
    SmartDashboard.putData(dispenserSubsystem);
    SmartDashboard.putData("Positioning", m_positioning);
    SmartDashboard.putData("AutoAim", m_autoaim); 

    // Adds autos to the autochooser

    autoChooser.setDefaultOption(" Center Shoot then taxi", new Right2NoteAuto(armSubsystem, dispenserSubsystem, m_autoaim, m_positioning, swerveSubsystem));
    autoChooser.addOption("taxi", new TaxiCommand(swerveSubsystem));
    autoChooser.addOption("Left Shoot then Taxi", new LeftAimAndShootAuto(armSubsystem, dispenserSubsystem, m_autoaim, m_positioning, swerveSubsystem));
    autoChooser.addOption("Right Shoot then Taxi", new RightAimAndShootAuto(armSubsystem, dispenserSubsystem, m_autoaim, m_positioning, swerveSubsystem));
    autoChooser.addOption("Shoots then backs up", new ScoreThenTaxi(armSubsystem, dispenserSubsystem, m_autoaim, m_positioning, swerveSubsystem));
    autoChooser.addOption("pathplannertest", new PathplannerTest(swerveSubsystem));

    autoChooser.addOption("Center 2 Note", new OneNoteAutoCenter(armSubsystem, dispenserSubsystem, m_autoaim, m_positioning, swerveSubsystem));
    autoChooser.addOption("Center 4 Note", new Center4NoteAuto(armSubsystem, dispenserSubsystem, m_autoaim, m_positioning, swerveSubsystem));
    autoChooser.addOption("Right 2 Note", new Right2NoteAuto(armSubsystem, dispenserSubsystem, m_autoaim, m_positioning, swerveSubsystem));
    //autoChooser.addOption("go back and forth lol", new GoStraight(armSubsystem, dispenserSubsystem, m_autoaim, m_positioning, swerveSubsystem)); //TODO probably remove


    SmartDashboard.putData(autoChooser);  
  }

  private void configureBindings() {

    // Driver Controller

    // Driver A: auto aligns and aims towards speaker
    driverController.a()
      .whileTrue(new YawToSpeaker(33,
                               () -> -driverController.getLeftY(),
                               () -> -driverController.getLeftX(),
                               m_positioning,
                               m_autoaim,
                               swerveSubsystem
                              ).repeatedly());


      //  driverController.a()
      // .whileTrue(new AlignToSpeaker(33,
      //                          () -> -driverController.getLeftY(),
      //                          () -> -driverController.getLeftX(),
      //                          m_positioning,
      //                          m_autoaim,
      //                          swerveSubsystem, armSubsystem
      //                         ).repeatedly());

    // Driver B: auto aligns and aims towards source TODO: test to see if this actually works                         
    driverController.b()
      .whileTrue(new AimAtSource(armSubsystem, 
                                dispenserSubsystem,
                                swerveSubsystem,
                                m_autoaim,
                                m_positioning,
                                () -> -driverController.getLeftY(),
                                () -> -driverController.getLeftX(),
                                driverController.rightTrigger()
                                ));

    // Drive Left Bumper: auto intakes
    driverController.leftBumper()
      .whileTrue(Commands.run(() -> dispenserSubsystem.autoIntake(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());

    // Driver Left Trigger: Spin up (hold to spin shooter motors set speed) //TODO DELETE THIS AT COMP THIS IS FOR TESTING ONLY
    driverController.leftTrigger(0.1)
      .whileTrue(Commands.run(() -> dispenserSubsystem.spinUpShooterWheels(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());

    // Driver Right Trigger: Shoot (pushes note into shooter wheels) //TODO DELETE THIS AT COMP THIS IS FOR TESTING ONLY
    driverController.rightTrigger(0.1)
      .whileTrue(Commands.run(() -> dispenserSubsystem.shootNoteImmediately(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());

    // Driver Back: go to setpoint //THIS IS ONLY FOR DEBUGGING
    driverController.back()
      .whileTrue(new RadiallyGoToAngle(33,
                                () -> -driverController.getLeftY(),
                                () -> -driverController.getLeftX(),
                                swerveSubsystem
                                ));

    // Driver Start: Reset gyro/field oriented
    driverController.start()
      .onTrue(new InstantCommand(() -> swerveSubsystem.resetGyro()));

    // // Pathplanner Testing
    // // Drive Left DPad: S forward
    // driverController.povRight()
    //   .onTrue(swerveSubsystem.sForward());

    // // Drive Left DPad: R forward
    // driverController.povLeft()
    //   .onTrue(swerveSubsystem.rForward());

    // // Drive Left DPad: R forward
    // driverController.povUp()
    //   .onTrue(swerveSubsystem.straightTurnPath());


    //Operator Controller: 


  // ARMSUBSYSTEM

    // Operator A: sets arm to 10 degrees(using PID)
    operatorController.a()
      .whileTrue(Commands.run(() -> armSubsystem.setArmSetpoint(45), armSubsystem))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

    // Operator X: sets arm to 55 degrees
    operatorController.x()
      .whileTrue(Commands.run(() -> armSubsystem.setArmSetpoint(90), armSubsystem))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

    // // Operator Y: Lock arm in place //TODO make this something more useful we only used this for debugging
    //  operatorController.y()
    //    .whileTrue(Commands.run(() -> armSubsystem.setVoltage(.7), armSubsystem))
    //    .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

    // Operator Dpad Up: Arm to 40 degrees (using PID)
    operatorController.povUp()
      .whileTrue(Commands.run(() -> armSubsystem.setArmSetpoint(15), armSubsystem))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

    // Operator Dpad Down: Arm to 30 degrees (using PID)
    operatorController.povDown()
      .whileTrue(Commands.run(() -> armSubsystem.setArmSetpoint(25), armSubsystem))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

    // Operator Dpad Left: Arm to amp angle (using PID) 
    operatorController.povLeft()
      .whileTrue(Commands.run(() -> armSubsystem.setArmSetpoint(20), armSubsystem))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

    // Operator Dpad Right: Arm to 20 degrees (using PID) 
    operatorController.povRight()
      .whileTrue(Commands.run(() -> armSubsystem.setArmSetpoint(10), armSubsystem))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

  //DISPENSER SUBSYSTEM

    // Operator B: Eject
    operatorController.b()
      .onTrue(Commands.run(() -> dispenserSubsystem.ejectNote(), dispenserSubsystem))        
      .onFalse(new InstantCommand(() -> dispenserSubsystem.stop(), dispenserSubsystem));
      
    //Operator y: line up ring 
    operatorController.y()   
      .whileTrue(new AutoAngleArm(33,
                    m_positioning,
                    m_autoaim,
                    armSubsystem
                  ).repeatedly());
 
    // Operator Left Trigger: Spin up (hold to spin shooter motors set speed)
    operatorController.leftTrigger(0.1)
      .whileTrue(Commands.run(() -> dispenserSubsystem.spinUpShooterWheels(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());

    // Operator Left Bumper: Auto Intake
    operatorController.leftBumper()
      .whileTrue(Commands.run(() -> dispenserSubsystem.autoIntake(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());

    // Operator Right Trigger: Shoot (pushes note into shooter wheels)
    operatorController.rightTrigger(0.1)
      .whileTrue(Commands.run(() -> dispenserSubsystem.shootNoteImmediately(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());
  }

  // Gets robot position for use in simulations
  public void simulationPeriodic(double period) {
    Pose2d pose = swerveSubsystem.getPose();
    m_positioning.simulationPeriodic(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  /** Runs continuously while robot is on */
  public void periodic() {

    // Update robot positioning
    m_positioning.update();

    // Update the LED pin
    Optional<Alliance> newAlliance = DriverStation.getAlliance();
    
    // Check if the alliance has changed (so we don't constantly set the pin)
    if (newAlliance.orElse(Alliance.Blue) != prevAlliance) {
      // Then set the pin HIGH for blue and LOW for red
      if (newAlliance.orElse(Alliance.Blue) == Alliance.Blue) {
        ledPin.set(true);
      } else {
        ledPin.set(false);
      }
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

