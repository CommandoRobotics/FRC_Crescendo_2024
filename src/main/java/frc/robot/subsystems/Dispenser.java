// Commando Robotics - FRC 5889
// Dispenser - Code for controlling the intake/outake (shooter) mechanism on our robot.

// This line states that the code in this file is part of our FRC Robot's package.
// The FRC package is something the RoboRio code looks for so it can run our code.
package frc.robot.subsystems;

// The imports include classes from various code libraries.
// They contain prewritten code we can use to make our job easier.
// The order does not affect the program, but we usually place the libraries from WPI first,
// and the place the others in alphabetical order, just so it is easy to read and conistent.
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// This class controls the internal electronics of the Dispenser as well as providing
// an interface for controlling it.
public class Dispenser extends SubsystemBase {
    // We usually start with variables that will be accessible/shared by all code in our class.
    // These variables are called "members" of our class. The WPI Lib settings encourage us to
    // begin member variable names with "m_" (i.e. m_someNumber).
    // Since other code might not know how to properly manipulate the variables in this class,
    // we usually mark the variables as "private". This prevents other classes from messing with
    // them.
    // Variables markes as "final" mean that its value cannot be changed after it is set. We can
    // use "final" to ensure we don't accidently overwrite values. If you try to change the value
    // of a variable marked as "final", the compiler will generate an error. Values for variables
    // marked as final can be set here. Other variables have to be set in the constructor.
    // Using "private" and "final" are not required, but using them in the appropriate situations
    // can help avoid improper access and prevent bugs in our code.

    // Declare all the motors that will be used in this class.
    private CANSparkMax m_intakeMotor;
    private CANSparkMax uppershooterMotor;
    private CANSparkMax lowershooterMotor;
     // Motor controlling the upper roller on the shooter.
    private DigitalInput m_intakeBeamBreak;
    private DigitalInput m_indexerBeamBreak;
    private DigitalInput m_shooterBeamBreak;
    
    // This special function is known as the constructor for the class. Notice that the constructor
    // has the EXACT SAME name (including capitalization) as our class' name. Also note that the
    // constructor does not have a return type (this is unlike other functions).
    // The constructor is where we set initial values and run any initial setup code.
    // Since our Robot class (in Robot.java) has a Dispenser as a member variable (m_dispenser),
    // this function will be called whenever our code starts: either on power-on, or when new
    // code is downloaded, or when the robot code is restarted following a code crash.
    public Dispenser() {
        // Set the information for our member variables.

        // Set the motors to the appropriate CAN device ID and set the type (brushed or brushless).
    
        m_intakeMotor = new CANSparkMax(41, MotorType.kBrushless);
        uppershooterMotor = new CANSparkMax(51, MotorType.kBrushless);
        lowershooterMotor = new CANSparkMax(53, MotorType.kBrushless);

        m_intakeBeamBreak = new DigitalInput(6);
        m_indexerBeamBreak = new DigitalInput(7);
        m_shooterBeamBreak = new DigitalInput(9);

        // Set the intake motor to "coast" (allow rotation) when we are not commanding them. This
        // will allow people to pull a note out of the intake when our code is not running.
        m_intakeMotor.setIdleMode(IdleMode.kCoast);
        uppershooterMotor.setIdleMode(IdleMode.kCoast);
        lowershooterMotor.setIdleMode(IdleMode.kCoast);
        // Set the motor as "inverted", meaining when we tell it to operate at a speed, it will
        // actually turn in reverse at that speed (i.e. -50% when we tell it +50%). This is because
        // we think of positive speeds as brining in a Note, but the motor is rotate such that positive
        // would normally spit the Note out.
        m_intakeMotor.setInverted(false);
        uppershooterMotor.setInverted(false);        
        
    }

    public Command stopCommand() {
        return run( () -> stop());
    }

    public Command spinCommand() {
        return run( () -> spinUpShooterWheels());
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
   
    // Return true if the Shooter beam brake sensor sees a note.
    boolean shooterDetectsNote() {
        boolean detecterSeesLight = m_shooterBeamBreak.get();
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
        m_intakeMotor.set(0.5);
        uppershooterMotor.set(0);
        lowershooterMotor.set(0);
        
    }

    // This function runs the intake motors until a Note is stored in our index area.
    // When we have a note stored, it turns off the intake motors so we do not accidently
    // intake another Note.
    public void autoIntake() {
        if (indexerDetectsNote()) {
            // Note is in indexer.
            m_intakeMotor.set(0);
        } else {
            // No Note yet
            intakeNote();
        }
    }

    // This function runs the motors in reverse to get rid of a note without shooting it.
    // An example of needing this is if we accidently intake a second Note.
    public void ejectNote() {
        m_intakeMotor.set(-0.5);
        uppershooterMotor.set(-0.5);
        lowershooterMotor.set(-0.5);
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
        m_intakeMotor.set(1.0);
        uppershooterMotor.set(1.0);
        lowershooterMotor.set(1.0);
        

    }

    // This function runs the intake motors to push the Note through the shooter, until the Note is fully out. Then it turns off the intake motors, but keeps the shooter motors spinning.
    public void feedShooter() {
        if (anySensorDetectsNote()) {
            shootNoteImmediately();
        }
    }

    // This function runs the shooter motors, while keeping the intake system stopped.
    // Since the shooter motors take some amount of time to get to their max speed, we would run
    // this function before we actually call shootNoteImmediately().
    public void spinUpShooterWheels() {
        
        m_intakeMotor.set(0);
        uppershooterMotor.set(1.0);
        lowershooterMotor.set(1.0);
        
    }

    // This function stops all the motors.
    // We'll use this function when have intaken Note, but are not ready to shoot it.
    public void stop() {
        m_intakeMotor.set(0);
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
