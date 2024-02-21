// Commando Robotics - FRC 5889
// Feeding - Code that commands robot to perform "arm up" operations.
// These operations are intaking from Source or placing in the Amp.
// This command automatically changes between the two.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.SwerveSubsystem;

// This command handles when the driver has the arm up, trying to score in Amp or Intake from Source.
// The driver controls the chassis x, y.
// The yaw (direction) is pointed toward the appropriate direction by logic in this command.
// The arm is commanded to appropriate height by logic in this command.
// The dispenser is controlled by logic in this command.
public class Feeding extends Command {
    Arm m_arm;
    Dispenser m_dispenser;
    SwerveSubsystem m_swerve;
    DoubleSupplier m_x;
    DoubleSupplier m_y;
    BooleanSupplier m_shoot;
    
    // This is the constructor, it stores references to the subsystems we will use.
    public Feeding(Arm the_arm, Dispenser the_dispenser, SwerveSubsystem the_swerve, DoubleSupplier the_x_translation, DoubleSupplier the_y_translation, BooleanSupplier the_shoot_indicator) {
        m_arm = the_arm;
        m_dispenser = the_dispenser;
        m_swerve = the_swerve;
        m_x = the_x_translation;
        m_y = the_y_translation;
        m_shoot = the_shoot_indicator;
    }

    // This will be called repeatedly while our system is running.
    @Override
    public void execute() {
        boolean ampIsCloser = true; // TODO: Call autoaim to determine whether we are closer to the source or the amp.

        // Perform appropriate arm/dispenser actions based on the element we are closer to.
        if (ampIsCloser) {
            amping();
        } else {
            sourcing();
        }
        
        // Drive the swerve drive
        double desiredAngle = 0; // TODO: Call autoaim to determine correct angle for the source
        drive(desiredAngle);
    }

    // Adjusts the robot so we can 
    public void sourcing() {
        // TODO: Call arm to set the appropriate angle for intaking from the source.

        // Call dispenser's autoIntake function.
        m_dispenser.autoIntake(); 
    }

    public void amping() {
        // TODO: Call arm to set the appropriate angle for intaking from the source.

        if (m_shoot.getAsBoolean() == true) {
            // Call dispenser's shootNoteImmdiately function.
        } else {
            // Call dispenser's spinUpShooter function so the shooter wheels are ready, but the Note does not get shot.
        }
    }

    public void drive(double desired_yaw) {
        // Drive slowly because we are near the intake/source.
        final double slow_factor = 0.5;
        double slow_x = slow_factor * m_x.getAsDouble();
        double slow_y = slow_factor * m_y.getAsDouble();
        m_swerve.driveCommand(() -> slow_x, () -> slow_y, () -> desired_yaw, true);
    }
}
