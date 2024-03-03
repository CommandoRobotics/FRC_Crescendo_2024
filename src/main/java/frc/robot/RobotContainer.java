// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dispenser;
import frc.robot.commands.AimingCommand;
import frc.robot.commands.FeedingCommand;
import frc.robot.commands.IntakingCommand;

public class RobotContainer {


  // Subsystems
  Arm armSubsystem = new Arm();
  Dispenser dispenserSubsystem = new Dispenser();
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // Other Utilities
  AutoAim m_autoaim = new AutoAim();
  Positioning m_positioning = new Positioning();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

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

    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData(armSubsystem);
    SmartDashboard.putData(dispenserSubsystem);
  }

  private void configureBindings() {

    //Driver Controller

    // Driver Left Trigger: Outake
    driverController.leftTrigger(0.1)
      .whileTrue(new InstantCommand(() -> dispenserSubsystem.ejectNote(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());

    // Driver Right Trigger: Intake  // TODO change to autointake if beanbreak works
    driverController.rightTrigger(0.1)
      .whileTrue(Commands.run(() -> dispenserSubsystem.autoIntake(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());

    //TODO actually do autoaim
    // Driver A: AutoAim Speaker

    // Driver B: AutoAim Source

    // Driver Start: Reset gyro/field oriented
    driverController.start().onTrue(new InstantCommand(() -> swerveSubsystem.resetGyro()));


    // Arm Controller

    // Operator A: Arm Flat (using limit switch

    operatorController.a()
      .onTrue(new InstantCommand(() -> armSubsystem.setAngleInDegrees(0), armSubsystem)
                  .andThen(new InstantCommand(() -> armSubsystem.autoControl(), armSubsystem).repeatedly()))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

    // Operator B: Eject
    operatorController.b()
      .onTrue(Commands.run(() -> dispenserSubsystem.ejectNote(), dispenserSubsystem))        
      .onFalse(new InstantCommand(() -> dispenserSubsystem.stop(), dispenserSubsystem));

    // Operator Y: Arm Up (using limit switch) //TODO might change to PID if we dont HAVE a limit switch
    operatorController.y()
      .whileTrue(Commands.run(() -> armSubsystem.setAngleInDegrees(90), armSubsystem))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

    // Operator Dpad Up: Arm Up (using PID) //TODO find full arm up if theres no limit switch
    operatorController.povUp()
      .whileTrue(new InstantCommand(() -> armSubsystem.setAngleInDegrees(90), armSubsystem)
                  .andThen(new InstantCommand(() -> armSubsystem.autoControl(), armSubsystem).repeatedly()))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

    // Operator Dpad Down: Arm Down (using PID) //TODO Could just use limit switch
    operatorController.povDown()
      .whileTrue(new InstantCommand(() -> armSubsystem.setAngleInDegrees(0), armSubsystem)
                  .andThen(new InstantCommand(() -> armSubsystem.autoControl(), armSubsystem).repeatedly()))
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

    // Operator Dpad Left: Go to Source height
    operatorController.povLeft()
      .whileTrue(armSubsystem.adjustTowardSourceCommand())
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

    // Operator Dpad Right: Go to Amp height
    operatorController.povRight()
      .whileTrue(armSubsystem.adjustTowardAmpCommand())
      .onFalse(new InstantCommand(() -> armSubsystem.stop(), armSubsystem));

    // Operator Left Trigger: Spin up (hold to spin shooter motors set speed)
    operatorController.leftTrigger(0.1)
      .whileTrue(new InstantCommand(() -> dispenserSubsystem.spinUpShooterWheels(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());

    // Operator Right Trigger: Shoot (pushes note into shooter wheels)
    operatorController.rightTrigger(0.1)
      .whileTrue(new InstantCommand(() -> dispenserSubsystem.shootNoteImmediately(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());

    // Operator Left Bumper: Auto Intake
    operatorController.leftBumper()
      .whileTrue(Commands.run(() -> dispenserSubsystem.autoIntake(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());


    // Operator Right Bumper: Auto Shoot (until empty)
    operatorController.rightBumper()
      .whileTrue(Commands.run(() -> dispenserSubsystem.feedShooter(), dispenserSubsystem))
      .onFalse(dispenserSubsystem.stopCommand());

    // Operator back: reset left encoder
    operatorController.back()
      .onTrue(Commands.runOnce(() -> armSubsystem.resetLeftEncoder(), armSubsystem));

      
    // Operator start: reset right encoder
    operatorController.start()
      .onTrue(Commands.runOnce(() -> armSubsystem.resetRightEncoder(), armSubsystem));
  
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
    // // Driver B: Auto aim robot for shot.
    // AimingCommand aim = new AimingCommand(
    //   m_arm,
    //   m_dispenser,
    //   swerveSubsystem,
    //   m_autoaim,
    //   m_positioning,
    //   () -> -driverController.getLeftY(),
    //   () -> -driverController.getLeftX(),
    //   driverController.rightTrigger()
    // );
    // driverController.b().whileTrue(aim);
  }

  public void simulationPeriodic(double period) {
    Pose2d pose = swerveSubsystem.getPose();
    m_positioning.simulationPeriodic(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    double x = 0.5;
    double y = 0.0;
    double headingX = 0.0;
    double headingY = 0.0;
    //double rotateRate = 0.0;
    return swerveSubsystem.driveCommand(
      () -> x,
      () -> y,
      () -> headingX,
      () -> headingY
    );
  }
}
