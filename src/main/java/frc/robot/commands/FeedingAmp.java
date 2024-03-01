// Commando Robotics - FRC 5889
// FeedingAmp - Code that handles mode where we want to dispense into the amp.
// Calls appropriate commands to face robot towards amp, raise arm appropriately,
// allows driver to command (slowly), and shoots when trigger is pulled.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.AutoAim;
import frc.robot.Constants.AutomationConstants;

/**
 * Mode where robot is pointed to amp with arm up, slows driver X speeds (to aid alignment), and lets driver shoot.
 */
public class FeedingAmp extends ParallelCommandGroup {
    
    // This is the constructor, it stores references to the subsystems we will use.
    public FeedingAmp(Arm theArm, Dispenser theDispenser, SwerveSubsystem theSwerve, AutoAim theAutoAim, DoubleSupplier xSpeed, DoubleSupplier ySpeed, BooleanSupplier allowedToShoot, boolean isBlueAlliance) {
        // Arm
        addCommands(theArm.adjustTowardAmpCommand());

        // Dispenser
        if (allowedToShoot.getAsBoolean()) {
            // Go ahead and dispense
            addCommands(theDispenser.ampDispenseCommand());
        } else {
            // Hold the note (do not let it slide out).
            addCommands(theDispenser.stopCommand());
        }
        
        // Chassis
        double desiredYawInDegrees = theAutoAim.getDesiredYawInDegreesToAmp(isBlueAlliance);
        // Drive slowly parallel to Amp, so we can align with the hole.
        double slowX = AutomationConstants.kAmpSpeedFactor * xSpeed.getAsDouble();
        // Drive normal speed perpendicular to amp, because the wall will stop us from driving too far.
        double normalY = ySpeed.getAsDouble();
        // YAGSL uses the X component and Y component of an angle to set the desired angle.
        Rotation2d desiredHeading = Rotation2d.fromDegrees(desiredYawInDegrees);
        double headingX = desiredHeading.getSin();
        double headingY = desiredHeading.getCos();
        addCommands(theSwerve.driveCommand(() -> slowX, () -> normalY, () -> headingX, () -> headingY));
    }
}
