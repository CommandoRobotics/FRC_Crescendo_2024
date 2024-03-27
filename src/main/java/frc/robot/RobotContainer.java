// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.API.AutoAim;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dispenser;
import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.AimAtSource;
import frc.robot.commands.AimAtSpeaker;
import frc.robot.commands.AimingCommand;
import frc.robot.commands.AlignToSpeaker;
import frc.robot.commands.FeedingCommand;
import frc.robot.commands.GoToYawSetpoint;
import frc.robot.commands.IntakingCommand;
import frc.robot.commands.LeftAimAndShootAuto;
import frc.robot.commands.RadiallyGoToAngle;
import frc.robot.commands.RightAimAndShootAuto;
import frc.robot.commands.ScoreThenTaxi;
import frc.robot.commands.TaxiCommand;

public class RobotContainer {


  // Subsystems
  Arm armSubsystem = new Arm();
  Dispenser dispenserSubsystem = new Dispenser();
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // Other Utilities
  AutoAim m_autoaim = new AutoAim();
  Positioning m_positioning = new Positioning();

  SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);


  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    //Start the driver camera
    CameraServer.startAutomaticCapture();
    
  
    
    //Default Commands

    //Default drive using the driver controller (field oriented)
    swerveSubsystem.setDefaultCommand(
      swerveSubsystem.driveCommand(() -> -MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
                                   () -> -MathUtil.applyDeadband(driverController.getLeftX(), 0.1), 
                                   () -> -MathUtil.applyDeadband(driverController.getRightX(), 0.1),
                                   true));

    //Spin the shooter wheels (ALWAYS :D) 
    //dispenserSubsystem.setDefaultCommand(dispenserSubsystem.spinCommand());
    
    //Manual control of all motors in the dispenser - operator right stick y EMERGENCY ONLY
    //dispenserSubsystem.setDefaultCommand((new InstantCommand(() -> dispenserSubsystem.setDispenser(operatorController.getLeftY()), dispenserSubsystem).repeatedly()));
    
    //Manual control of the arm - operator left stick Y
    armSubsystem.setDefaultCommand(new InstantCommand(() -> armSubsystem.manuallyPowerArmRestrained(-operatorController.getLeftY()), armSubsystem).repeatedly());


    dispenserSubsystem.setDefaultCommand(dispenserSubsystem.manualShootCommand(() -> -operatorController.getRightY()));

    // Configure the trigger bindings
    configureBindings();

    //dispenserSubsystem.setDefaultCommand(dispenserSubsystem.forceShootCommand());

    SmartDashboard.putData(armSubsystem);
    SmartDashboard.putData(dispenserSubsystem);
    SmartDashboard.putData("Positioning", m_positioning);
    SmartDashboard.putData("AutoAim", m_autoaim); 
    autoChooser.setDefaultOption(" Center Shoot then taxi", new AimAndShootCommand(armSubsystem, dispenserSubsystem, m_autoaim, m_positioning, swerveSubsystem));
    autoChooser.addOption("taxi", new TaxiCommand(swerveSubsystem));
    autoChooser.addOption("Left Shoot then Taxi", new LeftAimAndShootAuto(armSubsystem, dispenserSubsystem, m_autoaim, m_positioning, swerveSubsystem));
    autoChooser.addOption("Right Shoot then Taxi", new RightAimAndShootAuto(armSubsystem, dispenserSubsystem, m_autoaim, m_positioning, swerveSubsystem));
    autoChooser.addOption("Shoots then backs up", new ScoreThenTaxi(armSubsystem, dispenserSubsystem, m_autoaim, m_positioning, swerveSubsystem));

    SmartDashboard.putData(autoChooser);  


  }

  private void configureBindings() {

    //Driver Controller

    // Driver Left Trigger: Outake
    driverController.rightBumper()
      .whileTrue(new InstantCommand(() -> dispenserSubsystem.ejectNote(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());


    // Driver Left Trigger: Outake
    // driverController.y()
    //   .whileTrue(new InstantCommand(() -> swerveSubsystem.setGyro(-62.07), swerveSubsystem));

    // Driver Right Trigger: Intake  // TODO change to autointake if beanbreak works
   // driverController.rightTrigger(0.1)
    //  .whileTrue(Commands.run(() -> dispenserSubsystem.autoIntake(), dispenserSubsystem))
    //  .onFalse(dispenserSubsystem.stopCommand());

    //TODO actually do autoaim

    driverController.a()
      .whileTrue(new AlignToSpeaker(33,
                               () -> -driverController.getLeftY(),
                               () -> -driverController.getLeftX(),
                               m_positioning,
                               m_autoaim,
                               swerveSubsystem
                              ).repeatedly());


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

                               

    // Driver B: AutoAim Source

    // Driver Start: Reset gyro/field oriented
    driverController.start()
      .onTrue(new InstantCommand(() -> swerveSubsystem.resetGyro()));

    

    //driver intake
    driverController.leftBumper()
      .whileTrue(Commands.run(() -> dispenserSubsystem.autoIntake(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());


    // Driver back: go to setpoint
    driverController.back()
      .whileTrue(new RadiallyGoToAngle(33,
                               () -> -driverController.getLeftY(),
                               () -> -driverController.getLeftX(),
                                swerveSubsystem
                               ));




    // Arm Controller


    // Operator A: Arm Flat 

    operatorController.x()
      .whileTrue(Commands.run(() -> armSubsystem.setArmSetpoint(55), armSubsystem))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

    operatorController.a()
      .whileTrue(Commands.run(() -> armSubsystem.setArmSetpoint(10), armSubsystem))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));



      // Operator Y: Lock arm in place //TODO might change to PID if we dont HAVE a limit switch
     operatorController.y()
       .whileTrue(Commands.run(() -> armSubsystem.setVoltage(.7), armSubsystem))
       .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

    // Operator Dpad Up: Arm Up (using PID) //TODO find full arm up if theres no limit switch
    operatorController.povUp()
      .whileTrue(Commands.run(() -> armSubsystem.setArmSetpoint(40), armSubsystem))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

    // Operator Dpad Down: Arm shoot from subwoofer (using PID) //TODO Could just use limit switch
    operatorController.povDown()
      .whileTrue(Commands.run(() -> armSubsystem.setArmSetpoint(Constants.ArmConstants.kSubwooferAngle), armSubsystem))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

          // Operator Dpad Down: Arm Down (using PID) //TODO Could just use limit switch
    operatorController.povLeft()
      .whileTrue(Commands.run(() -> armSubsystem.setArmSetpoint(Constants.ArmConstants.kAmpAngle), armSubsystem))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

          // Operator Dpad Down: Arm Down (using PID) //TODO Could just use limit switch
    operatorController.povRight()
      .whileTrue(Commands.run(() -> armSubsystem.setArmSetpoint(20), armSubsystem))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));



      // Operator back: reset left encoder
    operatorController.back()
      .onTrue(Commands.runOnce(() -> armSubsystem.resetLeftEncoder(), armSubsystem));

      
    // Operator start: reset right encoder
    operatorController.start()
      .onTrue(Commands.runOnce(() -> armSubsystem.resetRightEncoder(), armSubsystem));

      
//DISPENSER SUBSYSTEM

     // Operator B: Eject
    operatorController.b()
      .onTrue(Commands.run(() -> dispenserSubsystem.ejectNote(), dispenserSubsystem))        
      .onFalse(new InstantCommand(() -> dispenserSubsystem.stop(), dispenserSubsystem));

    // Operator Left Trigger: Spin up (hold to spin shooter motors set speed)
    operatorController.leftTrigger(0.1)
      .whileTrue(Commands.run(() -> dispenserSubsystem.spinUpShooterWheels(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());

    // Operator Right Trigger: Shoot (pushes note into shooter wheels)
    operatorController.rightTrigger(0.1)
      .whileTrue(Commands.run(() -> dispenserSubsystem.shootNoteImmediately(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());


    // Operator Left Bumper: Auto Intake
    operatorController.leftBumper()
      .whileTrue(Commands.run(() -> dispenserSubsystem.autoIntake(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());


    
    




  
    // // Driver Modes
    // // Driver Start: Reset Gyro
    // driverController.start().onTrue(swerveSubsystem.resetGyroCommand());
    // // Driver Y : Place the arm up in feeding mode (from source or to amp).
    // FeedingCommand feed = new FeedingCommand(
    //   m_arm,
    //   m_dispenser,
    //   swerveSubsystem,
    //   m_autoaim,
    //   m_positioning,
    //   () -> -driverController.getLeftY(),
    //   () -> -driverController.getLeftX(),
    //   driverController.rightTrigger()
    // );
    // driverController.y().whileTrue(feed);
    // // Driver Right Trigger: Place the arm down in intaking mode.
    // IntakingCommand intake = new IntakingCommand(
    //   m_arm,
    //   m_dispenser,
    //   swerveSubsystem,
    //   () -> -driverController.getLeftY(),
    //   () -> -driverController.getLeftX(),
    //   () -> -driverController.getRightX()
    // );
    // driverController.rightTrigger().whileTrue(intake);
    // Driver B: Auto aim robot for shot.
  /*   AimingCommand aim = new AimingCommand(
      armSubsystem,
      dispenserSubsystem,
      swerveSubsystem,
      m_autoaim,
      m_positioning,
      () -> -driverController.getLeftY(),
      () -> -driverController.getLeftX(),
      driverController.rightTrigger()
    );
    driverController.b().whileTrue(aim);
  
*/
  }
  public void simulationPeriodic(double period) {
    Pose2d pose = swerveSubsystem.getPose();
    m_positioning.simulationPeriodic(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  public void periodic() {
    m_positioning.update();
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
