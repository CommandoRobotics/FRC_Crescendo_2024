// Commando Robotics - FRC 5889
// Arm - Code for controlling the arm (raises and lowers shooter/intake).
// Most of the code for this file comes from Rev's Alternate Encoder Example.
// https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Alternate%20Encoder/src/main/java/frc/robot/Robot.java

// This line states that the code in this file is part of our FRC Robot's package.
// The FRC package is something the RoboRio code looks for so it can run our code.
package frc.robot;

// The imports include classes from various code libraries.
// They contain prewritten code we can use to make our job easier.
// The order does not affect the program, but we usually place the libraries from WPI first,
// and the place the others in alphabetical order, just so it is easy to read and conistent.
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkLimitSwitch;


// This class controls the internal electronics of the arm as well as providing
// an interface for controlling it.
public class Arm {
    // Declare all the motors that will be used in this class.
    private CANSparkMax m_leftMotor; // Left side when you are looking toward the front of the robot.
    private CANSparkMax m_rightMotor; // Right side when you are looking toward the front of the robot.
    private DutyCycleEncoder m_hexBoreEncoder; // This is on the hex shaft of the arm.
    private final double m_encoderOffsetInRotations = 0.0; // Where the encoder actually reads when the arm is at zero degrees.
    private double m_setPointInDegrees; // Position the intake should move to/hold.

    // Constructor
    public Arm() {
        m_leftMotor = new CANSparkMax(31, MotorType.kBrushless);
        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor = new CANSparkMax(32, MotorType.kBrushless);
        m_rightMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setInverted(true);
        m_hexBoreEncoder = new DutyCycleEncoder(0); // Connected to this RoboRio DIO port.
        m_setPointInDegrees = 0.0;
    }

    // Call this if you want to manually control the arm motors.
    // Positive percentage is rotation from intake position (horiontal) to shooting (upright).
    // Percentage should be from -1.0 to +1.0.
    // DO NOT use autoControl and manual control at the same time.
    public void manuallyControlArm(double motorPercent) {
        m_leftMotor.set(motorPercent);
        m_rightMotor.set(motorPercent);
    }

    // Sets the target position of the Arm.
    // Arm in the intaking position (horizontal) is considered 0 degrees.
    public void setAngleInDegrees(double angleInDegrees) {
        m_setPointInDegrees = angleInDegrees;
    }

    // Call this every iteration to update the motor values.
    // Returns false if there is an issue with the arm.
    // DO NOT use autoControl and manual control at the same time.
    public boolean autoControl() {
        var currentAngleInDegrees = getAngleInDegrees();
        if (currentAngleInDegrees < 0 || currentAngleInDegrees > 180.0) {
            // We should not be here. Turn off motor power.
            m_leftMotor.set(0.0);
            m_rightMotor.set(0.0);
            // Allow motors to move freely in case it is jamming against something.
            m_leftMotor.setIdleMode(IdleMode.kCoast);
            m_rightMotor.setIdleMode(IdleMode.kCoast);
            return false;
        }
        // Determine how far we are from the desired rotation.
        double delta =  m_setPointInDegrees - currentAngleInDegrees;
        var powerValue = proportionalPower(delta);
        if (powerValue < 0.01) {
            // Close enough, stop the motor and maintain position.
            m_leftMotor.set(0.0);
            m_rightMotor.set(0.0);
            m_leftMotor.setIdleMode(IdleMode.kCoast);
            m_rightMotor.setIdleMode(IdleMode.kCoast);
        } else {
            m_leftMotor.set(powerValue);
            m_rightMotor.set(powerValue);
            m_leftMotor.setIdleMode(IdleMode.kCoast);
            m_rightMotor.setIdleMode(IdleMode.kCoast);
        }
        return true;
    }

    // Returns the current angle in degrees.
    // Range of values is 0 to 360.
    public double getAngleInDegrees() {
        double rotation = m_hexBoreEncoder.getAbsolutePosition() - m_encoderOffsetInRotations;
        // "Normalize" the value to the range of 0 to 1: adjust for negatives, or values greater than one rotation.
        while (rotation < 0) {
            rotation += 1.0;
        }
        while (rotation > 1.0) {
            rotation -= 1.0;
        }
        double rotationInDegrees = rotation * 360.0;
        return rotationInDegrees;
    }

    // Provides a scaled power value (-1.0 to +1.0) based on how far the motor needs to move.
    public double proportionalPower(double degreesToTravel) {
        // Handle positive and negative directions seperately because going down has gravity assisting.
        if (degreesToTravel > 45.0) {
            // More 45 degrees up, go full power.
            return 1.0;
        } else if (degreesToTravel > 15) {
            return 0.5; 
        } else if (degreesToTravel > 1.0) {
            return 0.25;
        } else if (degreesToTravel < -45.0) {
            return 0.25;
        } else if (degreesToTravel < -15.0) {
            return .125;
        } else if (degreesToTravel < -1.0) {
            return 0.5;
        } else {
            // Positions that are within one degree are close enough.
            return 0.0;
        }
    }
    
}
