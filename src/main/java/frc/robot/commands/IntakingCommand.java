// Commando Robotics - FRC 5889
// Intaking - Sets mode for collecting Notes from the floor.
// Arm is down, Intake is in auto mode (to prevent multiple Notes from coming in), and driver has control.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.SwerveSubsystem;

/*
 * Command for intake mode.
 */
public class IntakingCommand extends ParallelCommandGroup {

    public IntakingCommand(Arm theArm, Dispenser theDispenser, SwerveSubsystem theSwerve, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier yawSpeed) {
        addCommands(
            // Set the arm down (flat, 0 degrees)
            theArm.adjustTowardFloorCommand(),
            // Set the dispenser to auto intake. This will run the intake motors unless there is a note in the indexer.
            theDispenser.autoIntakeCommand(),
            // Let the driver drive.
            theSwerve.driveCommand(xSpeed, ySpeed, yawSpeed, true)
        );
    }
}
