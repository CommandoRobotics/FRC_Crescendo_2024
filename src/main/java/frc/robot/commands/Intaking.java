// Commando Robotics - FRC 5889
// Intaking - Code for command to drive the robot when the driver wants to intake.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.SwerveSubsystem;

// This command handles when the driver is trying to intake a Note.
// The driver controls the chassis x, y, and rotation.
// The arm is automatically commanded down.
// The shooter wheels are off, but intake is running (until indexer detects the Note).
public class Intaking extends Command {
    Arm m_arm;
    Dispenser m_dispenser;
    SwerveSubsystem m_swerve;
    DoubleSupplier m_x;
    DoubleSupplier m_y;
    DoubleSupplier m_yaw_rate;
    
    // This is the constructor, it stores references to the subsystems we will use.
    public Intaking(Arm the_arm, Dispenser the_dispenser, SwerveSubsystem the_swerve, DoubleSupplier the_x_translation, DoubleSupplier the_y_translation, DoubleSupplier the_yaw_rate) {
        m_arm = the_arm;
        m_dispenser = the_dispenser;
        m_swerve = the_swerve;
        m_x = the_x_translation;
        m_y = the_y_translation;
        m_yaw_rate = the_yaw_rate;
    }

    // This will be called repeatedly while our system is running.
    @Override
    public void execute() {
        // Set the arm down (flat, 0 degrees)
        // TODO: Call the appropraite function from m_arm

        // Set the dispenser to auto intake. This will run the intake motors unless there is a note in the indexer.
        // TODO: Call the appropriate function from m_dispenser to auto intake.
        
        // Drive the swerve drive
        m_swerve.driveCommand(m_x, m_y, m_yaw_rate, true);
    }
}
