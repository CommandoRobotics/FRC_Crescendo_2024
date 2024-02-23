// Commando Robotics - FRC 5889
// Auto Shoot - Command that runs the motors until the Note leaves the robot.
// This command does not signal finished until the note is fully out of the shooter.
// This is meant for Autonomous, because it does not allow the driver to control the robot while it is running.
// Usually the command after this would be to shoot.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.SwerveSubsystem;

// This command simply shoots the Note (but does not signal finished until the note is out).
public class AutoShoot extends Command {
    Arm m_arm;
    Dispenser m_dispenser;
    SwerveSubsystem m_swerve;
    boolean m_blueAlliance;
    boolean m_done;
    
    // This is the constructor, it stores references to the subsystems we will use.
    public AutoShoot(Arm the_arm, Dispenser the_dispenser, SwerveSubsystem the_swerve, boolean isBlueAlliance) {
        m_arm = the_arm;
        m_dispenser = the_dispenser;
        m_swerve = the_swerve;
        m_blueAlliance = isBlueAlliance;
        m_done = false;
    }

    // This is called each time the command is triggered.
    @Override
    public void initialize() {
        // Somebody called this command again, so reset this.
        m_done = false;
    }

    // This will be called repeatedly while our command is running.
    @Override
    public void execute() {
        // TODO: If the arm is empty, set m_done true and return

        // Keep shooting (since the note is not out yet.)
        m_dispenser.shootNoteImmediately();
    }

    // This indicates if we have finished shooting.
    // The command scheduler will keep calling our execute() until this returns true.
    @Override
    public boolean isFinished() {
        return m_done;
    }

}
