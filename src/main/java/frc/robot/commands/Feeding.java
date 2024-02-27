// Commando Robotics - FRC 5889
// Feeding - Code that commands robot to perform "arm up" operations.
// These operations are intaking from Source or placing in the Amp.
// This command automatically changes between the two.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.AutoAim;
import frc.robot.Positioning;

// This command handles when the driver has the arm up, trying to score in Amp or Intake from Source.
// The driver controls the chassis x, y.
// The yaw (direction) is pointed toward the appropriate direction by logic in this command.
// The arm is commanded to appropriate height by logic in this command.
// The dispenser is controlled by logic in this command.
public class Feeding extends Command {
    Arm m_arm;
    Dispenser m_dispenser;
    SwerveSubsystem m_swerve;
    AutoAim m_autoaim;
    Positioning m_positioning;
    DoubleSupplier m_x;
    DoubleSupplier m_y;
    BooleanSupplier m_shoot;
    boolean m_isBlueAlliance;
    double m_desiredYaw;
    
    // This is the constructor, it stores references to the subsystems we will use.
    public Feeding(Arm the_arm, Dispenser the_dispenser, SwerveSubsystem the_swerve, AutoAim the_autoaim, Positioning the_positioning, DoubleSupplier the_x_translation, DoubleSupplier the_y_translation, BooleanSupplier the_shoot_indicator, boolean on_blueAlliance) {
        m_arm = the_arm;
        m_dispenser = the_dispenser;
        m_swerve = the_swerve;
        m_autoaim = the_autoaim;
        m_positioning = the_positioning;
        m_x = the_x_translation;
        m_y = the_y_translation;
        m_shoot = the_shoot_indicator;
        m_isBlueAlliance = on_blueAlliance;
        m_desiredYaw = 0.0;
    }

    // This will be called repeatedly while our system is running.
    @Override
    public void execute() {
        Pose2d currentPose = m_positioning.getPose();
        boolean ampIsCloser = m_autoaim.isAmpCloser(currentPose.getX(), currentPose.getY());

        // Perform appropriate arm/dispenser actions and yaw based on the element we are closer to.
        if (ampIsCloser) {
            amping();
        } else {
            sourcing();
        }
        // Drive the swerve, slowly.
        drive(m_desiredYaw);
    }

    // Adjusts the robot so we can 
    public void sourcing() {
        // Set arm to angle for intaking from the source.
        m_arm.setSourceIntake();
        // Run the dispenser's intake (auto stop when we have a Note).
        m_dispenser.autoIntake();
        // Set the yaw so we point at the Source.
        m_desiredYaw = m_autoaim.getDesiredYawInDegreesToSource(m_isBlueAlliance);
    }

    public void amping() {
        // Set arm to angle for scoring in the Amp.
        m_arm.setAmpArngle();
        // If the driver said we can shoot, do so. Otherwise just spin up the output motors.
        if (m_shoot.getAsBoolean() == true) {
            // Trigger pulled. Shoot immediately.
            m_dispenser.shootNoteImmediately();
        } else {
            // Call dispenser's spinUpShooter function so the shooter wheels are ready, but the Note does not get shot.
            m_dispenser.spinUpShooterWheels();
        }
        // Set the yaw so we point at the Amp.
        m_desiredYaw = m_autoaim.getDesiredYawInDegreesToAmp(m_isBlueAlliance);
    }

    public void drive(double desired_yaw) {
        // Drive slowly because we are near the intake/source.
        final double slow_factor = 0.5;
        double slow_x = slow_factor * m_x.getAsDouble();
        double slow_y = slow_factor * m_y.getAsDouble();
        m_swerve.driveCommand(() -> slow_x, () -> slow_y, () -> desired_yaw, true);
    }
}
