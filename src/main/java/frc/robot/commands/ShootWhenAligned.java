// Commando Robotics - FRC 5889
// Shoot When Aligned - Command that aligns the robot and exits when the robot is aligned enough to shoot.
// This is meant for Autonomous, because it does not allow the driver to control the robot while it is running.
// Usually the command after this would be to shoot.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.SwerveSubsystem;

// This command lines up (chassis yaw and arm pitch) to the speaker and then shoots.
public class ShootWhenAligned extends SequentialCommandGroup {
    
    // For a SequentialCommandGroup, the constructor is all that is needed.
    public ShootWhenAligned(Arm the_arm, Dispenser the_dispenser, SwerveSubsystem the_swerve, boolean isBlueAlliance) {
        // Add a list of commands. When one signals isFinished, the next will run.
        addCommands(
            new AutoAlign(the_arm, the_dispenser, the_swerve, isBlueAlliance),
            new AutoShoot(the_arm, the_dispenser, the_swerve, isBlueAlliance)
        );
    }
}
