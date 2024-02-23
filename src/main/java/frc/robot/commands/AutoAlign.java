// Commando Robotics - FRC 5889
// Auto Align - Command that aligns the robot and exits when the robot is aligned enough to shoot.
// This is meant for Autonomous, because it does not allow the driver to control the robot while it is running.
// Usually the command after this would be to shoot.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.SwerveSubsystem;

// This command lines up (chassis yaw and arm pitch) to the speaker and then signals finished.
public class AutoAlign extends Command {
    Arm m_arm;
    Dispenser m_dispenser;
    SwerveSubsystem m_swerve;
    // TODO: Merge Positioning code
    //Positioning m_positioning;
    boolean m_blueAlliance;
    boolean m_done;
    
    // This is the constructor, it stores references to the subsystems we will use.
    public AutoAlign(Arm the_arm, Dispenser the_dispenser, SwerveSubsystem the_swerve, boolean isBlueAlliance) {
        m_arm = the_arm;
        m_dispenser = the_dispenser;
        m_swerve = the_swerve;
        m_blueAlliance = isBlueAlliance;
        // TODO: Merge positioning code
        //m_positioning = new Positioning;
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
        // TODO: Get latest update from positioning.
        //positioning.update()
        
        // TODO: Get the current yaw
        //positioning.getYaw()

        // Get the current pitch of the arm.
        Rotation2d armPosition = m_arm.getCurrentArmPosition();
        double armPositionInDegrees = armPosition.getDegrees();

        // TODO: Use the auto aim code to determine if we are aligned enough
        //boolean alignedEnough = m_autoAim.alignedEnoughToShoot();
        // if (alignedEnough) {
        //   m_done = true;
        //   return true;
        // }

        // If we made it here, we are not aligned.
        // TODO: Slowly align. Do not drive forwards/backwards
        //m_swerve.drive(0, 0, autoAim.desired(), true);
    }

    // This indicates if we have finished shooting.
    // The command scheduler will keep calling our execute() until this returns true.
    @Override
    public boolean isFinished() {
        return m_done;
    }

}
