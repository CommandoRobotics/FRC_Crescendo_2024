// Commando Robotics - FRC 5889
// Arm - Code for controlling the arm (raises and lowers shooter/intake).
// Most of the code for this file comes from Rev's Alternate Encoder Example.
// https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Alternate%20Encoder/src/main/java/frc/robot/Robot.java

// This line states that the code in this file is part of our FRC Robot's package.
// The FRC package is something the RoboRio code looks for so it can run our code.
package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
// The imports include classes from various code libraries.
// They contain prewritten code we can use to make our job easier.
// The order does not affect the program, but we usually place the libraries from WPI first,
// and the place the others in alphabetical order, just so it is easy to read and conistent.
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkLimitSwitch;


// This class controls the internal electronics of the arm as well as providing
// an interface for controlling it.
public class Arm implements Sendable {
    // Declare all the motors that will be used in this class.
    private CANSparkMax m_leftMotor; // Left side when you are looking toward the front of the robot.
    private CANSparkMax m_rightMotor; // Right side when you are looking toward the front of the robot.
    
    // Declare the Encoder
    private DutyCycleEncoder m_hexBoreEncoder; // This is on the hex shaft of the arm.
    private final Rotation2d m_encoderOffset = Rotation2d.fromRotations(0.0); // TODO: Determine actual encoddr offset.
    private Rotation2d m_desiredAngle; // Variable to sore where the arm should move to/hold.

    private final ArmFeedforward m_armFeedFoward = new ArmFeedforward(0, 0, 0, 0);
    private final PIDController m_armPID = new PIDController(12, 1e-4, 0.5); // TODO: Tune this PID

    // The following is just for simulation and debugging
    private SingleJointedArmSim m_simulatedArm;
    private final double armLengthInMeters = 1.0; // TDOD: Update this with correct length.
    private final double armMassInKilograms = 20.0; // TDOD: Update this with correct mass.
    private final double ammReduction = 5 * 3 * 6; // 3 stage of 5:1 and a 6:1 chain reduction.
    private final Rotation2d minPosition = Rotation2d.fromDegrees(0);
    private final Rotation2d maxPosition = Rotation2d.fromDegrees(135);
    private DutyCycleEncoderSim m_simulatedEncoder;
    private double m_debuggingLastPIDOutput;
    private double m_debbugingLastFeedForwardOutput;
    private boolean m_testUp;

    // Constructor
    public Arm() {
        m_leftMotor = new CANSparkMax(31, MotorType.kBrushless);
        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor = new CANSparkMax(32, MotorType.kBrushless);
        m_rightMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setInverted(true);
        m_hexBoreEncoder = new DutyCycleEncoder(0); // Connected to this RoboRio DIO port.
        m_desiredAngle = Rotation2d.fromDegrees(0.0);

        m_simulatedArm = new SingleJointedArmSim(
            DCMotor.getNEO(2),
            ammReduction,
            SingleJointedArmSim.estimateMOI(armLengthInMeters, armMassInKilograms),
            armLengthInMeters,
            minPosition.getRadians(),
            maxPosition.getRadians(),
            true,
            minPosition.getRadians()
        );
        m_simulatedEncoder = new DutyCycleEncoderSim(m_hexBoreEncoder);
        m_debbugingLastFeedForwardOutput = 0;
        m_debuggingLastPIDOutput = 0.0;
        m_testUp = true;
    }

    // Sets the motors to brake mode and stop them.
    public void stop() {
        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setIdleMode(IdleMode.kBrake);

        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
    }

    // Call this if you want to manually control the arm motors.
    // Positive percentage is rotation from intake position (horiontal) to shooting (upright).
    // Percentage should be from -1.0 to +1.0.
    // DO NOT use autoControl and manual control at the same time.
    public void manuallyPowerArm(double motorPercent) {
        m_leftMotor.set(motorPercent);
        m_rightMotor.set(motorPercent);
    }

    // Sets the target position of the Arm.
    // Arm in the intaking position (horizontal) is considered 0 degrees.
    public void setAngleInDegrees(double angleInDegrees) {
        m_desiredAngle = Rotation2d.fromDegrees(angleInDegrees);
    }

    public Rotation2d getCurrentArmPosition() {
        // Read the value from the encoder.
        Rotation2d readingAngle = Rotation2d.fromRotations(m_hexBoreEncoder.get());
        // The encoder's zero value does not always match the arm's zero, so apply the offest.
        Rotation2d actualAngle = readingAngle.minus(m_encoderOffset);
        return actualAngle;
    }

    // Call this every iteration to update the motor values.
    // Returns false if there is an issue with the arm.
    // DO NOT use autoControl and manual control at the same time.
    public boolean autoControl() {
        double feedForwardOutput = m_armFeedFoward.calculate(m_desiredAngle.getRadians(), 1.0);
        m_debbugingLastFeedForwardOutput = feedForwardOutput;
        double pidOutput = m_armPID.calculate(getCurrentArmPosition().getRadians(), m_desiredAngle.getRadians());
        m_debuggingLastPIDOutput = pidOutput;
        double totalMotorOutput = feedForwardOutput + pidOutput;
        m_leftMotor.set(totalMotorOutput);
        m_rightMotor.set(totalMotorOutput);
        return true;
    }

    public void simulationPeriodic(double timeSinceLastCall) {
        m_simulatedArm.setInput(m_leftMotor.get());
        //m_simulatedArm.setInput(m_leftMotor.get() * RobotController.getBatteryVoltage());
        m_simulatedArm.update(timeSinceLastCall);
        m_simulatedEncoder.setDistance(m_simulatedArm.getAngleRads());
        // RoboRioSim.setVInVoltage(
        //     BatterySim.calculateDefaultBatteryLoadedVoltage(
        //         m_simulatedArm.getCurrentDrawAmps()
        //     )
        // );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("ArmModule");
      builder.addDoubleProperty("armCurrentDegrees", this::dashboardGetCurrentArmPositionInDegrees, null);
      builder.addDoubleProperty("armDesiredDegrees", this::dashboardGetDesiredtArmPositionInDegrees, null);
      builder.addDoubleProperty("motorCurrentVoltage", this::dashboardGetArmVoltage, null);
      builder.addDoubleProperty("FeedForwardOutput", this::dashboardGetLastFeedForwardOutput, null);
      builder.addDoubleProperty("PIDOutput", this::dashboardGetLastPIDOutput, null);
    }

    double dashboardGetDesiredtArmPositionInDegrees() {
        return m_desiredAngle.getDegrees();
    }

    double dashboardGetCurrentArmPositionInDegrees() {
        return getCurrentArmPosition().getDegrees();
    }

    double dashboardGetArmVoltage() {
        return m_leftMotor.get() * 12.0;
    }

    double dashboardGetLastPIDOutput() {
        return m_debuggingLastPIDOutput;
    }

    double dashboardGetLastFeedForwardOutput() {
        return m_debbugingLastFeedForwardOutput;
    }

    // Test raising and lowering the arm
    // To run this test, call it in autonomousPeriodic
    public void test() {
        double current = m_desiredAngle.getDegrees();
        if (m_testUp && current >= 89.9) {
            // Already at top, go down.
            m_testUp = false;
            setAngleInDegrees(0);
        } else if (current < 0.1) {
            // Already at bottom, go up.
            m_testUp = true;
            setAngleInDegrees(90);
        }
        autoControl();
    }
}
