// Commando Robotics - FRC 5889
// FeedingSource - Code that handles mode where we want to intake from source (human player).
// Calls appropriate commands to face robot towards source, raise arm appropriately, dispenser to intake, and
// allows driver to command (slowly).

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DispenserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.API.AutoAim;
import frc.robot.Constants.AutomationConstants;

/**
 * Mode where robot is pointed to amp with arm up, slows driver X speeds (to aid alignment), and lets driver shoot.
 */
public class FeedingSource extends ParallelCommandGroup {
    
    // This is the constructor, it stores references to the subsystems we will use.
    public FeedingSource(ArmSubsystem theArm, DispenserSubsystem theDispenser, SwerveSubsystem theSwerve, AutoAim theAutoAim, DoubleSupplier xSpeed, DoubleSupplier ySpeed, boolean isBlueAlliance) {
        // Arm
        addCommands(theArm.adjustTowardSourceCommand());

        // Dispenser
        addCommands(theDispenser.autoIntakeCommand());
        
        // Chassis
        double desiredYawInDegrees = theAutoAim.getDesiredYawInDegreesToSource(isBlueAlliance);
        // Drive slowly so we can align with the slot.
        double slowX = AutomationConstants.kSourceSpeedFactor * xSpeed.getAsDouble();
        double slowY = AutomationConstants.kSourceSpeedFactor * ySpeed.getAsDouble();
        // YAGSL uses the X component and Y component of an angle to set the desired angle.
        Rotation2d desiredHeading = Rotation2d.fromDegrees(desiredYawInDegrees);
        double headingX = desiredHeading.getSin();
        double headingY = desiredHeading.getCos();
        addCommands(theSwerve.driveCommand(() -> slowX, () -> slowY, () -> headingX, () -> headingY));
    }
}
