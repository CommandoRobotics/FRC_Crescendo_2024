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
public class Dispenser extends SubsystemBase {

    // Declare all the motors that will be used in this class.
    private CANSparkMax m_intakeMotor;
    private CANSparkMax uppershooterMotor;
    private CANSparkMax lowershooterMotor;

    //Declares the sensors used in this class
    private DigitalInput m_intakeBeamBreak;
    private DigitalInput m_indexerBeamBreak;
    private DigitalInput m_shooterBeamBreak;
    

    public Dispenser() {

    // Set the motors to the appropriate CAN device ID and set the type (brushed or brushless).   
        m_intakeMotor = new CANSparkMax(41, MotorType.kBrushless);
        uppershooterMotor = new CANSparkMax(53, MotorType.kBrushless);
        lowershooterMotor = new CANSparkMax(51, MotorType.kBrushless);

        m_intakeBeamBreak = new DigitalInput(DispenserConstants.kRioDIOPortIntakeBeamBreak);
        m_indexerBeamBreak = new DigitalInput(DispenserConstants.kRioDIOPortIndexerBeamBreak);
        m_shooterBeamBreak = new DigitalInput(DispenserConstants.kRioDIOPortShooterBeamBreak);

        // Set the intake motor to "coast" (allow rotation) when we are not commanding them. This
        // will allow people to pull a note out of the intake when our code is not running.
        m_intakeMotor.setIdleMode(IdleMode.kCoast);
        uppershooterMotor.setIdleMode(IdleMode.kCoast);
        lowershooterMotor.setIdleMode(IdleMode.kCoast);

        //set motor inversions and amp limits
        m_intakeMotor.setInverted(false);
        uppershooterMotor.setInverted(false);
        m_intakeMotor.setSmartCurrentLimit(40);
        uppershooterMotor.setSmartCurrentLimit(40);
        lowershooterMotor.setSmartCurrentLimit(40);
    }

    // Sets zero speed, but has motors hold position.
    public Command stopCommand() {
        return run(() -> stop());
    }

    // Sets zero power and allows motors to spin freely.
    public Command releaseCommand() {
        return run(() -> release());
    }

    //allows for manual control of the shooter
    public Command manualSpinCommand(DoubleSupplier threshold) {
        return run( () -> spinThreshold(threshold.getAsDouble()));
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
        m_intakeMotor.set(speed);
        lowershooterMotor.set(speed);
        uppershooterMotor.set(speed);
    }

    // Return true if the Intake beam brake sensor sees a note.
    boolean intakeDetectsNote() {
        // The intake sensor is high (true) when it sees the beam (light).
        // When the beam is "broken" (Note blocking the light), the sensor is low (false).
        boolean detectorSeesLight = m_intakeBeamBreak.get();
        return !detectorSeesLight; // If the detector sees the beam, there is no Note.
    }
   
    // Return true if the Indexer beam brake sensor sees a note.
    boolean indexerDetectsNote() {
        boolean beamIsBroken = !m_indexerBeamBreak.get();
        return beamIsBroken;
    }

    public void spinThreshold(double threshold) {
        m_intakeMotor.set(0);
        uppershooterMotor.set(threshold);
        lowershooterMotor.set(threshold);
    }
   
    // Return true if the Shooter beam brake sensor sees a note.
    boolean shooterDetectsNote() {
        boolean detecterSeesLight = !m_shooterBeamBreak.get();
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

    // This function runs the motors to pull in a Note (but not shoot it yet).
    public void intakeNote() {
        // Turn the intake wheels at 50% (0.5) speed.
        intakeAtSpeed(DispenserConstants.kFloorIntakeSpeed);
    }

    // Runs intake at the specified speed of 0 (stop) to 1 (full speed)
    public void intakeAtSpeed(double speed) {
        m_intakeMotor.set(speed);
        uppershooterMotor.set(0);
        lowershooterMotor.set(0);

    }

    // This function runs the intake motors until a Note is stored in our index area.
    // When we have a note stored, it turns off the intake motors so we do not accidently
    // intake another Note.
    public void autoIntake() {
      if (indexerDetectsNote() && !intakeDetectsNote() ) {
        m_intakeMotor.set(-0.2);
    } else if ((shooterDetectsNote() && indexerDetectsNote()) || shooterDetectsNote()) {
        ejectNote();
    } else if (indexerDetectsNote() && !shooterDetectsNote()) {
        // Note is in indexer.
        m_intakeMotor.set(0);
    }  else {
        // No Note yet
        intakeNote();
    }
    }

    // This function runs the motors in reverse to get rid of a note without shooting it.
    // An example of needing this is if we accidently intake a second Note.
    public void ejectNote() {
        m_intakeMotor.set(-0.4);
        uppershooterMotor.set(-0.4);
        lowershooterMotor.set(-0.4);
    }

    public void ejectNoteSlow() {
        m_intakeMotor.set(-0.1);
        uppershooterMotor.set(-0.1);
        lowershooterMotor.set(-0.1);
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
        // Since the Note is still in contact with the intake wheels, we need to use
        // them to move the Note along our Dispenser, into the shooter wheels.
        // Help the shooter by running intake as quickly as we can, so it will start to speed
        // up the Note before it hits the shooter wheels. The Note will only be in contact
        // with the shooter wheels for a split second, so if the intake has the Note moving
        // quickly, the shooter wheels can make it go even faster. But if the Note was traveling
        // slowly through the intake, it won't come out of the shooter as fast as possible.
        // So, set the intake speed to 100%.
        m_intakeMotor.set(Constants.DispenserConstants.kShooterSubwooferSpeed);
        uppershooterMotor.set(Constants.DispenserConstants.kShooterSubwooferSpeed);
        lowershooterMotor.set(Constants.DispenserConstants.kShooterSubwooferSpeed);
    }

    // This function runs the intake motors to push the Note through the shooter, until the Note is fully out. Then it turns off the intake motors, but keeps the shooter motors spinning.
    public void feedShooter() {
        if (anySensorDetectsNote()) {
            shootNoteImmediately();
        }
    }

    //revs up shooter
    public void spinUpShooterWheels() {
        // Stop the intake (and force brake mode so a Note can't roll out).
        m_intakeMotor.set(0);
        m_intakeMotor.setIdleMode(IdleMode.kBrake);
        uppershooterMotor.set(DispenserConstants.kShooterIdleSpeed);
        lowershooterMotor.set(DispenserConstants.kShooterIdleSpeed);
        
    }


    //revs up the shooter slowly
    public void spinUpShooterWheelsSlow() {
        // Stop the intake (and force brake mode so a Note can't roll out).
        m_intakeMotor.set(0);
        m_intakeMotor.setIdleMode(IdleMode.kBrake);
        uppershooterMotor.set(0.07);
        lowershooterMotor.set(0.07);
        
    }

    // This function stops all the motors.
    // We'll use this function when have intaken Note, but are not ready to shoot it.
    public void stop() {
        m_intakeMotor.set(0);
        uppershooterMotor.set(0);
        lowershooterMotor.set(0);
        // Ensure motors do not roll out on their own.
        m_intakeMotor.setIdleMode(IdleMode.kBrake);

        
    }

    // Set motor powers to zero and coast mode.
    // Useful before/after matches so we can get notes in or out by had.
    public void release() {
        m_intakeMotor.set(0);
        uppershooterMotor.set(0);
        lowershooterMotor.set(0);
        // Allow motors to roll freely.
        m_intakeMotor.setIdleMode(IdleMode.kCoast);
        uppershooterMotor.setIdleMode(IdleMode.kCoast);
        lowershooterMotor.setIdleMode(IdleMode.kCoast);
    }

    // The following sends information about this subsystem to the Smart Dashboard.
    @Override
    public void periodic() {
        // The arm is perpendicular to the Upright shoulder.
        SmartDashboard.putBoolean("intake detects note", intakeDetectsNote());
        SmartDashboard.putBoolean("indexer detects note", indexerDetectsNote());
        SmartDashboard.putBoolean("shooter detects note", shooterDetectsNote());

    }
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
        return m_intakeBeamBreak.get();
    }

    public boolean dashboardGetIndexerBeamBreak() {
        return m_indexerBeamBreak.get();
    }

    public boolean dashboardGetShooterBeamBreak() {
        return m_shooterBeamBreak.get();
    }

    public double dashboardGetIntakeMotorPower() {
        return m_intakeMotor.get();
    }

    public double dashboardGetShooterUpperMotorPower() {
        return uppershooterMotor.get();
    }

    public double dashboardGetShooterLowerMotorPower() {
        return lowershooterMotor.get();
    }
}
