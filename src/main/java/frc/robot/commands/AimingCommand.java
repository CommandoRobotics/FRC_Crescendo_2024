// Commando Robotics - FRC 5889
// Aiming - Sets mode for traveling to speaker to shoot.
// Arm is targeting speaker, dispenser is waiting for trigger, yaw is targeting spaker, and
// driver has control of X/Y.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AutomationConstants;
import frc.robot.Positioning;
import frc.robot.API.AutoAim;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DispenserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/*
 * Command for intake mode.
 */
public class AimingCommand extends ParallelCommandGroup {

    public AimingCommand(ArmSubsystem theArm, DispenserSubsystem theDispenser, SwerveSubsystem theSwerve, AutoAim theAutoAim, Positioning thePositioning, DoubleSupplier xSpeed, DoubleSupplier ySpeed, BooleanSupplier allowedToShoot) {
        // Determine which alliance we are, so we know where our own Speaker is.
        boolean isBlueAlliance = true;
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                isBlueAlliance = false;
            }
        }
        thePositioning.update();
        Pose2d currentPose = thePositioning.getPose();

        // Arm
        double desiredPitch = theAutoAim.getDesiredShooterAngleInDegrees(currentPose.getX(), currentPose.getY(), isBlueAlliance);
        addCommands(theArm.setpointCommand(() -> desiredPitch));

        // Dispenser
        if (allowedToShoot.getAsBoolean()) {
            // Go ahead and shoot
            addCommands(theDispenser.shootUntilEmptyCommand());
        } else {
            // Hold the note (do not let it slide out).
            addCommands(theDispenser.stopCommand());
        }
        
        // Chassis
        double desiredYawInDegrees = theAutoAim.getDesiredYawInDegreesToSpeaker(currentPose.getX(), currentPose.getY(), currentPose.getRotation(), theSwerve.getYaw(), isBlueAlliance);
        // YAGSL uses the X component and Y component of an angle to set the desired angle.
        Rotation2d desiredHeading = Rotation2d.fromDegrees(desiredYawInDegrees);
        double headingX = desiredHeading.getSin();
        double headingY = desiredHeading.getCos();
        addCommands(theSwerve.driveCommand(xSpeed, ySpeed, () -> headingX, () -> headingY));
    }
}
