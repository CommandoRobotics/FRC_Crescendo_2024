// Commando Robotics - FRC 5889
// Dispenser - Code for controlling the intake/outake (shooter) mechanism on our robot.

// This line states that the code in this file is part of our FRC Robot's package.
// The FRC package is something the RoboRio code looks for so it can run our code.
package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.DispenserConstants;

// This class controls the internal electronics of the Dispenser as well as providing
// an interface for controlling it.
public class DispenserSubsystem extends SubsystemBase {

    // Declare all the motors that will be used in this class.
    private CANSparkMax intakeMotor;
    private CANSparkMax uppershooterMotor;
    private CANSparkMax lowershooterMotor;

    //Declares the sensors used in this class
    private DigitalInput intakeBeamBreak;
    private DigitalInput indexerBeamBreak;
    private DigitalInput shooterBeamBreak;
    

    public DispenserSubsystem() {

      // Set the motors to the appropriate CAN device ID and set the type (brushed or brushless).   
      intakeMotor = new CANSparkMax(41, MotorType.kBrushless);
      uppershooterMotor = new CANSparkMax(53, MotorType.kBrushless);
      lowershooterMotor = new CANSparkMax(51, MotorType.kBrushless);

      intakeBeamBreak = new DigitalInput(DispenserConstants.kRioDIOPortIntakeBeamBreak);
      indexerBeamBreak = new DigitalInput(DispenserConstants.kRioDIOPortIndexerBeamBreak);
      shooterBeamBreak = new DigitalInput(DispenserConstants.kRioDIOPortShooterBeamBreak);

      // Set the intake motor to "coast" (allow rotation) when we are not commanding them. This
      // will allow people to pull a note out of the intake when our code is not running.
      intakeMotor.setIdleMode(IdleMode.kBrake);
      uppershooterMotor.setIdleMode(IdleMode.kCoast);
      lowershooterMotor.setIdleMode(IdleMode.kCoast);

      // Set motor inversions and current limits
      intakeMotor.setInverted(false);
      uppershooterMotor.setInverted(false);
      intakeMotor.setSmartCurrentLimit(40);
      uppershooterMotor.setSmartCurrentLimit(40);
      lowershooterMotor.setSmartCurrentLimit(40);
    }

    // Sets zero speed, but has motors hold position.
    public Command stopCommand() {
        return run(() -> stop());
    }

    //revs up the shooter 
    public Command spinCommand() {
        return run(() -> spinUpShooterWheels());
    }

    //intakes w/ beambreaks
    public Command autoIntakeCommand() {
        return run(() -> autoIntake());
    }

    //shoots whatevers in the robot
    public Command forceShootCommand() {
        return run(() -> feedShooter());
    }

    //shoots until empty
    public Command shootUntilEmptyCommand() {
        return run(() -> feedShooter());
    }

    //ejects whatevers in the robot out the intake
    public Command ejectNoteCommand() {
        return run(() -> ejectNote());
    }

    //sets speed to put note in the amp
    public Command ampDispenseCommand() {
        return run(() -> setDispenser(DispenserConstants.kAmpDispenseSpeed));
    }

    // Speed should be in the range 0 (stop) to 1 (shoot full power)
    public Command dispenseAtSpeedCommand(DoubleSupplier speed) {
        return run(() -> setDispenser(speed.getAsDouble()));
    }

    //sets speed of the intake at the source
    public Command sourceIntakeCommand() {
        return run(() -> intakeAtSpeed(DispenserConstants.kSourceIntakeSpeed));
    }

    // Speed should be in the range 0 (stop) to 1 (shoot full power)
    public Command intakeAtSpeedCommand(DoubleSupplier speed) {
        return run(() -> intakeAtSpeed(speed.getAsDouble()));
    }


    public Command manualShootCommand(DoubleSupplier speed) {
        return run( () -> setDispenser(speed.getAsDouble()));
    }

    /**
     * Sets all the dispenser motors to the given speed
     * @param speed
     */
    public void setDispenser(double speed) {
        // Only allow positive speeds, from 0 to 1 (full power).
        intakeMotor.set(speed);
        lowershooterMotor.set(speed);
        uppershooterMotor.set(speed);
    }

    // Return true if the Intake beam brake sensor sees a note.
    boolean intakeDetectsNote() {
        boolean detectorSeesLight = intakeBeamBreak.get();
        return !detectorSeesLight; // If the detector sees the beam, there is no Note.
    }
   
    // Return true if the Indexer beam brake sensor sees a note.
    boolean indexerDetectsNote() {
        boolean beamIsBroken = !indexerBeamBreak.get();
        return beamIsBroken;
    }
   
    // Return true if the Shooter beam brake sensor sees a note.
    boolean shooterDetectsNote() {
        boolean detecterSeesLight = !shooterBeamBreak.get();
        return detecterSeesLight;
    }

    // This function returns true if there is a note anywhere in the dispenser,
    // or false if it is empty.
    // Although each of the "...DetectsNote()" functions could be called individually,
    // this single function helps simplify code where we need to know about all of the
    // beam break sensors at once.
    boolean anySensorDetectsNote() {
        if (intakeDetectsNote()) {
            return true; // Note in intake
        } else if (indexerDetectsNote()) {
            return true; // Note in indexer
        } else if (shooterDetectsNote()) {
            return true; // Note in shooter
        } else {
            return false; // No Note detected
        }
    }

    /** Sets the intake motor to the predetermined intake speed */
    public void intakeNote() {
        intakeAtSpeed(DispenserConstants.kFloorIntakeSpeed);
    }

    /** Set the intake motor to the given speed and stops the shooter motors */
    public void intakeAtSpeed(double speed) {
        intakeMotor.set(speed);
        uppershooterMotor.set(0);
        lowershooterMotor.set(0);
    }

    /**
     * Runs the intake motor and automatically positions the note in the index depending 
     * on the beam break sensors <p/>
     * 
     * Will intake until the indexer BB detects the note and the intake note doesn't, and 
     * then will eject the note slowly until both the indexer and intake BB are triggered
     * (Which is determined to be the best place for the note to be)
     */
    public void autoIntake() {
      if (indexerDetectsNote() && !intakeDetectsNote() ) {
        intakeMotor.set(-0.2);
      } else if ((shooterDetectsNote() && indexerDetectsNote()) || shooterDetectsNote()) {
        ejectNote();
      } else if (indexerDetectsNote() && !shooterDetectsNote()) {
         // Note is in indexer.
         intakeMotor.set(0);
      }  else {
         // No Note yet
         intakeNote();
      }
    }

    /** 
     * This function runs the motors in reverse to get rid of a note without shooting it.
     * An example of needing this is if we accidently intake a second Note.
     */
    public void ejectNote() {
        intakeMotor.set(-0.4);
        uppershooterMotor.set(-0.4);
        lowershooterMotor.set(-0.4);
    }

    // This function runs all the motors in reverse if there is a Note anywhere in the dispenser. When there are no notes in the dispenser, it turns off the intake and resumes spinning up the shooter motors.
    void ejectUntilEmpty() {
        if (anySensorDetectsNote()) {
            // Still have a Note, keep trying to eject.
            ejectNote();
        } else {
            // No notes in the dispenser, so we can stop ejecting.
            spinUpShooterWheels();
        }
    }

    // This function turns on all the motors to shoot the Note.
    public void shootNoteImmediately() {
        intakeMotor.set(Constants.DispenserConstants.kShooterSubwooferSpeed);
        uppershooterMotor.set(Constants.DispenserConstants.kShooterSubwooferSpeed);
        lowershooterMotor.set(Constants.DispenserConstants.kShooterSubwooferSpeed);
    }

    // This function runs the intake motors to push the Note through the shooter, until the Note is fully out. Then it turns off the intake motors, but keeps the shooter motors spinning.
    public void feedShooter() {
        if (anySensorDetectsNote()) {
            shootNoteImmediately();
        }
    }

    /** Revs the shooter wheels */
    public void spinUpShooterWheels() {
        // Stop the intake (and force brake mode so a Note can't roll out).
        intakeMotor.set(0);
        uppershooterMotor.set(DispenserConstants.kShooterIdleSpeed);
        lowershooterMotor.set(DispenserConstants.kShooterIdleSpeed);
    }

    /** Stops all dispenser motors */
    public void stop() {
        intakeMotor.set(0);
        uppershooterMotor.set(0);
        lowershooterMotor.set(0);
    }

    // The following sends information about this subsystem to the Smart Dashboard.
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("Dispenser");
        builder.addDoubleProperty("intakePower", this::dashboardGetIntakeMotorPower, null);
        builder.addDoubleProperty("shooterUppperPower", this::dashboardGetShooterUpperMotorPower, null);
        builder.addDoubleProperty("shooterLowerPower", this::dashboardGetShooterLowerMotorPower, null);
        builder.addBooleanProperty("intakeBeam", this::dashboardGetIntakeBeamBreak, null);
        builder.addBooleanProperty("indexerBeam", this::dashboardGetIndexerBeamBreak, null);
        builder.addBooleanProperty("shooterBeam", this::dashboardGetShooterBeamBreak, null);
    }

    public boolean dashboardGetIntakeBeamBreak() {
        return intakeBeamBreak.get();
    }

    public boolean dashboardGetIndexerBeamBreak() {
        return indexerBeamBreak.get();
    }

    public boolean dashboardGetShooterBeamBreak() {
        return shooterBeamBreak.get();
    }

    public double dashboardGetIntakeMotorPower() {
        return intakeMotor.get();
    }

    public double dashboardGetShooterUpperMotorPower() {
        return uppershooterMotor.get();
    }

    public double dashboardGetShooterLowerMotorPower() {
        return lowershooterMotor.get();
    }

    @Override
    public void periodic() {
        // The arm is perpendicular to the Upright shoulder.
        SmartDashboard.putBoolean("intake detects note", intakeDetectsNote());
        SmartDashboard.putBoolean("indexer detects note", indexerDetectsNote());
        SmartDashboard.putBoolean("shooter detects note", shooterDetectsNote());
    }
}
