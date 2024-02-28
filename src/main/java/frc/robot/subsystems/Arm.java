// Commando Robotics - FRC 5889
// Arm - Code for controlling the arm (raises and lowers shooter/intake).
// Most of the code for this file comes from Rev's Alternate Encoder Example.
// https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Alternate%20Encoder/src/main/java/frc/robot/Robot.java

// This line states that the code in this file is part of our FRC Robot's package.
// The FRC package is something the RoboRio code looks for so it can run our code.
package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

import frc.robot.Constants.ArmConstants;


// This class controls the internal electronics of the arm as well as providing
// an interface for controlling it.
public class Arm extends SubsystemBase {
    // Declare all the motors that will be used in this class.
    private CANSparkMax m_leftMotor; // Left side when you are looking toward the front of the robot.
    private CANSparkMax m_rightMotor; // Right side when you are looking toward the front of the robot.
    
    // Declare the Encoder
    private DutyCycleEncoder m_leftHexBoreEncoder; // This is on the hex shaft of the arm.
    private DutyCycleEncoder m_rightHexBoreEncoder; // This is on the hex shaft of the arm.
    private final Rotation2d m_leftEncoderOffset = Rotation2d.fromRotations(ArmConstants.kLeftArmEncoderOffsetInRotations);
    private final Rotation2d m_rightEncoderOffset = Rotation2d.fromRotations(ArmConstants.kRightArmEncoderOffsetInRotations);
    private Rotation2d m_desiredAngle; // Variable to sore where the arm should move to/hold.

    private final ArmFeedforward m_armFeedFoward = new ArmFeedforward(0, 0, 0, 0);
    private final PIDController m_armPID = new PIDController(12, 1e-4, 0.5); // TODO: Tune this PID
    
    // Added two limit switches DI
    private DigitalInput m_upLimitSwitch;
    private DigitalInput m_downLimitSwitch;
    // The following is just for simulation and debugging
    private SingleJointedArmSim m_simulatedArm;
    private final double armLengthInMeters = 0.77; // Updated with correct length.
    private final double armMassInKilograms = 20.0; // TDOD: Update this with correct mass.
    private final double armReduction = 5 * 4 * 2 * 3; // Max planetary 5:1, 4:1, 2:1 and a 3:1 chain reduction.
    private final Rotation2d minPosition = Rotation2d.fromDegrees(0);
    private final Rotation2d maxPosition = Rotation2d.fromDegrees(90);
    private DutyCycleEncoderSim m_simulatedEncoder;
    private double m_debuggingLastPIDOutput;
    private double m_debbugingLastFeedForwardOutput;
    private double m_debuggingLastCommandedTotalMotorOutput;
    private boolean m_testUp;
    private boolean m_detectedLeftEncoderBad;
    private boolean m_detectedRightEncoderBad;
    private MechanismLigament2d m_supportLigament;
    private MechanismLigament2d m_armLigament;

    // Constructor
    public Arm() {
        m_leftMotor = new CANSparkMax(31, MotorType.kBrushless);
        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor = new CANSparkMax(32, MotorType.kBrushless);
        m_rightMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setInverted(true);
        m_leftHexBoreEncoder = new DutyCycleEncoder(0); // Connected to this RoboRio DIO port.
        m_rightHexBoreEncoder = new DutyCycleEncoder(1); // Connected to this RoboRio DIO port.
        m_desiredAngle = Rotation2d.fromDegrees(0.0);
        m_upLimitSwitch = new DigitalInput(2);
        m_downLimitSwitch = new DigitalInput(3);
        
        m_simulatedArm = new SingleJointedArmSim(
            DCMotor.getNEO(2),
            armReduction,
            SingleJointedArmSim.estimateMOI(armLengthInMeters, armMassInKilograms),
            armLengthInMeters,
            minPosition.getRadians(),
            maxPosition.getRadians(),
            true,
            minPosition.getRadians()
        );
        m_simulatedEncoder = new DutyCycleEncoderSim(m_leftHexBoreEncoder);
        m_debbugingLastFeedForwardOutput = 0;
        m_debuggingLastPIDOutput = 0.0;
        m_testUp = true;
        m_debuggingLastCommandedTotalMotorOutput = 0.0;
        m_detectedLeftEncoderBad = false;
        m_detectedRightEncoderBad = false;
        Mechanism2d mech = new Mechanism2d(3, 3);
        MechanismRoot2d root = mech.getRoot("shooter", 1, 0);
        m_supportLigament = root.append(new MechanismLigament2d("support", .28, 90, 6, new Color8Bit(Color.kGray)));
        m_armLigament =
            m_supportLigament.append(
                new MechanismLigament2d("arm", armLengthInMeters, 45, 6, new Color8Bit(Color.kBlue)
            )
        );
        SmartDashboard.putData("Mech2d", mech);
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

    // Uses the preset angle for intkaing from the floor.
    public void setFloorIntake() {
        setAngleInDegrees(ArmConstants.kFloorIntakeAngle);
    }

    public void setSourceIntake() {
        setAngleInDegrees(ArmConstants.kSourceAngle);
    }

    public void setAmpArngle() {
        setAngleInDegrees(ArmConstants.kAmpAngle);
    }

    // Returns the angle the left Arm is at.
    public Rotation2d getLeftPosition() {
        Rotation2d measuredAngle = Rotation2d.fromRotations(m_leftHexBoreEncoder.get());
        return getAdjustedPosition(measuredAngle, m_leftEncoderOffset, ArmConstants.kLeftArmReversed);
    }

    // Returns the angle the right Arm is at.
    public Rotation2d getRightPosition() {
        Rotation2d measuredAngle = Rotation2d.fromRotations(m_rightHexBoreEncoder.get());
        return getAdjustedPosition(measuredAngle, m_rightEncoderOffset, ArmConstants.kRightArmReversed);
    }

    // Determines actual angle, adjusting for the encoder's offest and whether it is reversed.
    public Rotation2d getAdjustedPosition(Rotation2d readingAngle, Rotation2d offset, boolean reversed) {
        // The encoder's zero value does not always match the arm's zero, so apply the offset.
        Rotation2d actualAngle = readingAngle.minus(offset);
        double actualAngleInDegrees = actualAngle.getDegrees();

        if (reversed) {
            // After adjusting for the offest, the zero angle should be correct
            // but a reversed encoder will decrease as the arm raises.
            // This means it goes from 0 degrees to -90 degrees (when the arm is upright).
            // We can just multiply by negative one to get a positive value.
            actualAngleInDegrees = -1 * actualAngleInDegrees;
        }
        return Rotation2d.fromDegrees(actualAngleInDegrees);       
    }

    // Returns false if the left encoder is returning a value outside the arm's range of motion (0-110 degrees).
    public boolean leftEncoderReasonable() {
        if (armEncoderReasonable(getLeftPosition())) {
            return true;
        } else {
            m_detectedLeftEncoderBad = true;
            return false;
        }
    }

    // Returns false if the right encoder is returning a value outside the arm's range of motion (0-110 degrees).
    public boolean rightEncoderReasonable() {
        if (armEncoderReasonable(getRightPosition())) {
            return true;
        } else {
            m_detectedRightEncoderBad = true;
            return false;
        }
    }

    public boolean armEncoderReasonable(Rotation2d measuredPosition) {
        // Arm should not go (much) below horizontal.
        if (measuredPosition.getDegrees() < 0) {
            return false;
        }
        // Arm can't go (much) past 90 degrees, unless it were broken.
        if (measuredPosition.getDegrees() > 95) {
            return false;
        }
        // Didn't fail the checks above, so seems reasonable.
        return true;
    }

    public Rotation2d getCurrentArmPosition() {
        if (leftEncoderReasonable()) {
            return getLeftPosition();
        } else {
            return getRightPosition();
        }
    }

    public boolean getDownLimitSwitchPressed() {
        boolean isLimitSwitchPressed = !m_downLimitSwitch.get();
        return isLimitSwitchPressed;        
    }

    public boolean getUpLimitSwitchPressed() {
        boolean isLimitSwitchPressed = !m_upLimitSwitch.get();
        return isLimitSwitchPressed;
    }

    // Call this every iteration to update the motor values.
    // Returns false if there is an issue with the arm.
    // DO NOT use autoControl and manual control at the same time.
    public boolean autoControl() {
        double feedForwardOutput = m_armFeedFoward.calculate(getCurrentArmPosition().getRadians(), 1.0);
        m_debbugingLastFeedForwardOutput = feedForwardOutput;
        double pidOutput = m_armPID.calculate(getCurrentArmPosition().getRadians(), m_desiredAngle.getRadians());
        m_debuggingLastPIDOutput = pidOutput;
        double totalMotorOutput = feedForwardOutput + pidOutput;
        // Make sure we do not set the motors beyond what they can actually do.
        totalMotorOutput = MathUtil.clamp(totalMotorOutput, -1.0, 1.0);
        m_debuggingLastCommandedTotalMotorOutput = totalMotorOutput;
        if (totalMotorOutput > 0 && getUpLimitSwitchPressed()) {
            totalMotorOutput = 0;
        } else if (totalMotorOutput < 0 && getDownLimitSwitchPressed()) {
            totalMotorOutput = 0;
        }
        m_leftMotor.set(totalMotorOutput);
        m_rightMotor.set(totalMotorOutput);
        
        return true;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    // This is called once per scheduler run, but only during simulation.
    public void simulationPeriodic() {
        m_simulatedArm.setInput(m_leftMotor.get() * RobotController.getBatteryVoltage());
        // Default period is 20 milliseconds
        m_simulatedArm.update(.02);
        m_simulatedEncoder.setDistance(m_simulatedArm.getAngleRads());
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                m_simulatedArm.getCurrentDrawAmps()
            )
        );

        m_armLigament.setAngle(getCurrentArmPosition().getDegrees());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("ArmModule");
      builder.addDoubleProperty("armCurrentDegrees", this::dashboardGetCurrentArmPositionInDegrees, null);
      builder.addDoubleProperty("armDesiredDegrees", this::dashboardGetDesiredtArmPositionInDegrees, null);
      builder.addDoubleProperty("motorCurrentVoltage", this::dashboardGetArmVoltage, null);
      builder.addDoubleProperty("FeedForwardOutput", this::dashboardGetLastFeedForwardOutput, null);
      builder.addDoubleProperty("PIDOutput", this::dashboardGetLastPIDOutput, null);
      builder.addDoubleProperty("commandedMotorOutput", this::dashboardGetLastCommandedTotalMotorOutput, null);
      builder.addDoubleProperty("batteryVoltage", this::dashboardGetBatteryVoltage, null);
      builder.addBooleanProperty("leftGood", this::dashboardGetLeftArmGood, null);
      builder.addBooleanProperty("rightGood", this::dashboardGetRightArmGood, null);
      builder.addBooleanProperty("topLimit", this::getUpLimitSwitchPressed, null);
      builder.addBooleanProperty("bottomLimit", this::getDownLimitSwitchPressed, null);
    }

    double dashboardGetDesiredtArmPositionInDegrees() {
        return m_desiredAngle.getDegrees();
    }

    double dashboardGetCurrentArmPositionInDegrees() {
        return getCurrentArmPosition().getDegrees();
    }

    double dashboardGetArmVoltage() {
        return m_leftMotor.get() * 12;
        //return m_leftMotor.get() * RobotController.getBatteryVoltage();
    }

    double dashboardGetLastPIDOutput() {
        return m_debuggingLastPIDOutput;
    }

    double dashboardGetLastFeedForwardOutput() {
        return m_debbugingLastFeedForwardOutput;
    }

    double dashboardGetLastCommandedTotalMotorOutput() {
        return m_debuggingLastCommandedTotalMotorOutput;
    }

    double dashboardGetBatteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    boolean dashboardGetLeftArmGood() {
        return !m_detectedLeftEncoderBad;
    }

    boolean dashboardGetRightArmGood() {
        return !m_detectedRightEncoderBad;
    }

    // Test raising and lowering the arm
    // To run this test, call it in autonomousPeriodic
    public void test() {
        double current = getCurrentArmPosition().getDegrees();
        if (m_testUp && (current >= 89.9)) {
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

    public void upTest() {
        setAngleInDegrees(90);
        autoControl();
    }
}
