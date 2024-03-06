// Commando Robotics - FRC 5889
// Feeding - Code that automatically sets robot into either Amping or Sourcing modes.
// This exists for convenience so driver just has one button when they put the arm up.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Positioning;
import frc.robot.API.AutoAim;

public class FeedingCommand extends ParallelCommandGroup {    
    // This is the constructor, it stores references to the subsystems we will use.
    public FeedingCommand(Arm theArm, Dispenser theDispenser, SwerveSubsystem theSwerve, AutoAim theAutoAim, Positioning thePositioning, DoubleSupplier xSpeed, DoubleSupplier ySpeed, BooleanSupplier allowedToShoot) {
        // Determine which alliance we are, so we know where our own Amp/Source are.
        boolean isBlueAlliance = true;
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                isBlueAlliance = false;
            }
        }
        Pose2d currentPose = thePositioning.getPose();

        if(theAutoAim.isAmpCloser(currentPose.getX(), currentPose.getY())){
            addCommands(
                new FeedingAmp(
                    theArm,
                    theDispenser,
                    theSwerve,
                    theAutoAim,
                    xSpeed,
                    ySpeed,
                    allowedToShoot,
                    isBlueAlliance
                )
            );
        } else {
            addCommands(
                new FeedingSource(
                    theArm,
                    theDispenser,
                    theSwerve,
                    theAutoAim,
                    xSpeed,
                    ySpeed,
                    isBlueAlliance
                )
            );
        }
    }
}
