package frc.robot.API;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.FieldElements;
import frc.robot.Constants.FieldLines;

// Handles calculations of robot/arm rotation when given current location.
public class AutoAim implements Sendable {
    boolean m_yawAligned = false; // Stores whether the last calculation deterimened yaw was good enough to shoot.
    boolean m_pitchAligned = false; // Stores whether the last calculation deterimened arm was good enough to shoot.

    double m_debugDeltaX; // For debugging only: X Distance from center of robot to target.
    double m_debugDeltaY; // For debugging only: Y Distance from center of robot to target.
    double m_debugTargetYawInDegrees; // For debugging only: Y Distance from center of robot to target.
    double m_debugErrorYawInDegrees; // For debugging only: Y Distance from center of robot to target.
    double m_debugDesiredYawInDegrees; // For debugging only: Y Distance from center of robot to target.
    //10.58.89.1
    // Returns the yaw to turn to based on current position.
    // Inputs are based on the WPILib Blue coordinate system.

    public double getDesiredYawInDegreesToSpeaker(double currentX, double currentY, Rotation2d currentLimelightYaw, Rotation2d currentGyroYaw, boolean isBlueAlliance) {
        // Set blue by default
        double targetX = Constants.FieldElements.kBlueSpeakerX;
        double targetY = Constants.FieldElements.kBlueSpeakerY;
        if (!isBlueAlliance) {
            targetX = Constants.FieldElements.kRedSpeakerX;
            targetY = Constants.FieldElements.kRedSpeakerY;
        }
        // Compared to current location, find meters in X,Y from current position to target.
        // Note: This may result in a negative value (meaning opposite direction).
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        m_debugDeltaX = deltaX;
        m_debugDeltaY = deltaY;

        Rotation2d targetYaw = Rotation2d.fromRadians(Math.atan(deltaY/deltaX));
        m_debugTargetYawInDegrees = targetYaw.getDegrees();
        Rotation2d yawError = currentGyroYaw.minus(currentLimelightYaw);
        m_debugErrorYawInDegrees = yawError.getDegrees();
        Rotation2d desiredYaw = targetYaw.plus(yawError);
        m_debugDesiredYawInDegrees = desiredYaw.getDegrees();
        if (isBlueAlliance) {
            return 180 + desiredYaw.getDegrees();
            
        } else {

        
        return desiredYaw.getDegrees();
        }
    }

    // This function determines what angle we should turn to if we want to "shoot" into the Amp.
    public double getDesiredYawInDegreesToAmp(boolean isBlueAlliance) {
        // Both amps are at 90° according to WPI Blue Origin
        return 90.0;
    }

    // This function determines what angle we should turn to if we want to "intake" from the Source.
    public double getDesiredYawInDegreesToSource(boolean isBlueAlliance) {
        if (isBlueAlliance) {
            return 300.0;
        } else {
            return 240.0;
        }
    }

    // This function returns if we are closer to the Amp than the speaker.
    // This function will be used to help determine what we want to do when the Driver says arm up.
    public boolean isAmpCloser(double currentX, double currentY) {
        // Just approximate by determinig whether we are on the Amp (audience) side, or Source (Scoring Table) side.

        final double fieldWidthInMeters  = 8.21;
        final double fieldSplit = fieldWidthInMeters / 2;
        if (currentY < fieldSplit) {
            // Origin is on source side
            return false;
        } else {
            // Further side from origin is Amp side.
            return true;
        }
    }

    // This function returns the correct direction to turn if the driver has the arm fully up.
    // When the arm is fully up, we are either intaking from the Source, or placing in the Amp.
        // This automatically switches the rotation of the robot based on the closest of the two.
        public double getDesiredYawInDegreesIfArmUp(double currentX, double currentY, boolean isBlueAlliance) {
            if (isAmpCloser(currentX, currentY)) {
                return getDesiredYawInDegreesToAmp(isBlueAlliance);
            } else {
                return getDesiredYawInDegreesToSource(isBlueAlliance);
            }
        }

    // Returns the estimated shooter angle in degrees.
    public double getDesiredShooterAngleInDegrees(double currentX, double currentY, boolean isBlueAlliance){
        double displacementFromSpeaker = shootingDistanceInMetersToSpeaker(currentX, currentY, isBlueAlliance);
        System.out.println("we are " + displacementFromSpeaker + "meters from the speaker");
        double desiredAngle = Math.atan(Constants.FieldElements.kSpeakerHeight / displacementFromSpeaker); //finds our desired angle based on the height of the speaker and our displacement from the speaker
        double desiredAngleDegrees = desiredAngle*(180/Math.PI);
        return desiredAngleDegrees;
    }

    // Returns true if we are not in the opposing alliance's wing.
    public boolean legalToShoot(double currentX, double currentY, boolean isBlueAlliance){ //returns true if we are not in opposing alliance's wing
        boolean isLegalToShoot;
        double bufferInMeters = 1.5; // Make sure entire body and bumpers are clear (even if rotated at an angle), and the referee can clearly see it.
        if(isBlueAlliance == false && currentX < (Constants.FieldLines.kBlueWingLineX - bufferInMeters)){//returns false if we are red alliance and in Blue ALliance's wing
            isLegalToShoot = false;
        } 
        else if (isBlueAlliance == true && currentX >= (Constants.FieldLines.kRedWingLineX + bufferInMeters)){   //returns false if we are blue alliance and in Red Alliance's wing
            isLegalToShoot = false;
        }   
        else {
            isLegalToShoot = true;
        }
            return isLegalToShoot;
    }

    // Returns the current distance from the speaker.
    // This can be used to help determine whether a shot is within range.
    public double shootingDistanceInMetersToSpeaker(double currentX, double currentY, boolean isBlueAlliance) {
        // Initially assume Blue Alliance
        double targetX = Constants.FieldElements.kBlueSpeakerX;
        double targetY = Constants.FieldElements.kBlueSpeakerY;
        // Change target if Red Alliance (NOT blue).
        if (!isBlueAlliance) {
            targetX = Constants.FieldElements.kRedSpeakerX;
            targetY = Constants.FieldElements.kRedSpeakerY;
        }
        
        double xDistance = targetX - currentX; //calculates x distance from speaker in meters
        double yDistance = targetY - currentY; //calculates y distance from speaker in meters

        double displacementFromSpeaker = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
        return displacementFromSpeaker;
    }

    // This function determines if we should "pull the trigger", our our robot is not ready.
    // x: X coordinates in Blue origin field.
    // y: Y coordinates in Blue origin field.
    // yawInDegres: Robot's current yaw.
    // armAngleInDegres: Position of arm.
    public boolean alignedEnoughToShoot(double currentX, double currentY, Rotation2d currentLimelightYaw, Rotation2d currentGyroYaw, double armAngleInDegrees, boolean isBlueAlliance) {
        double distanceInMeters = shootingDistanceInMetersToSpeaker(currentX, currentY, isBlueAlliance);
        
        // Determine if the robot is turned to an acceptable angle.
        // This could theoretically be calculated, but we with wind resistance and floppiness of the
        // Note, we will just manually test and determine how much variation is allowed.
        // We will add cases to this if-statement as we test.
        double allowedYawErrorInDegrees = 0;
        if (distanceInMeters < 2) {
            allowedYawErrorInDegrees = 5.0;
        } else if (distanceInMeters < 5) {
            allowedYawErrorInDegrees = 2;
        } else if (distanceInMeters < 10) {
            allowedYawErrorInDegrees = 1;
        } else {
            allowedYawErrorInDegrees = 0.1;
        }

        double desiredYawInDegrees = getDesiredYawInDegreesToSpeaker(currentX, currentY, currentLimelightYaw, currentGyroYaw, isBlueAlliance);
        double yawError = Math.abs(desiredYawInDegrees - currentGyroYaw.getDegrees());
        if (yawError < allowedYawErrorInDegrees) {
            // Close enought
            m_yawAligned = true;
        } else {
            m_yawAligned = false;
        }

        // Determine if the arm is at an acceptable angle
        double allowedArmErrorInDegrees = 0;
        if (distanceInMeters < 2) {
            allowedArmErrorInDegrees = 5.0;
        } else if (distanceInMeters < 5) {
            allowedArmErrorInDegrees = 2;
        } else if (distanceInMeters < 10) {
            allowedArmErrorInDegrees = 1;
        } else {
            allowedArmErrorInDegrees = 0.1;
        }

        double desiredArmAngleInDegrees = getDesiredShooterAngleInDegrees(currentX, currentY, isBlueAlliance);
        double pitchError = Math.abs(desiredArmAngleInDegrees - armAngleInDegrees);
        
        if (pitchError < allowedArmErrorInDegrees) {
            m_pitchAligned = true;
        } else {
            m_pitchAligned = false;
        }

        if (m_yawAligned && m_pitchAligned) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("AutoAim");
        builder.addBooleanProperty("yawGood", this::dashboardYawIsAligned, null);
        builder.addBooleanProperty("pitchGood", this::dashboardPitchIsAligned, null);
        builder.addDoubleProperty("deltaX", this::dashboardDeltaX, null);
        builder.addDoubleProperty("deltaY", this::dashboardDeltaY, null);
        builder.addDoubleProperty("targetYaw", this::dashboardTargetYaw, null);
        builder.addDoubleProperty("errorYaw", this::dashboardErrorYaw, null);
        builder.addDoubleProperty("desiredYaw", this::dashboardDesiredYaw, null);
    }

    private boolean dashboardYawIsAligned() {
        return m_yawAligned;
    }

    private boolean dashboardPitchIsAligned() {
        return m_pitchAligned;
    }

    private double dashboardDeltaX() {
        return m_debugDeltaX;
    }

    private double dashboardDeltaY() {
        return m_debugDeltaY;
    }

    private double dashboardTargetYaw() {
        return m_debugTargetYawInDegrees;
    }

    private double dashboardErrorYaw() {
        return m_debugErrorYawInDegrees;
    }

    private double dashboardDesiredYaw() {
        return m_debugDesiredYawInDegrees;
    }

}
