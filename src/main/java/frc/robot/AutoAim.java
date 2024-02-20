package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;

// Handles calculations of robot/arm rotation when given current location.
public class AutoAim implements Sendable {
    boolean yawAligned = false; // Stores whether the last calculation deterimened yaw was good enough to shoot.
    boolean pitchAligned = false; // Stores whether the last calculation deterimened arm was good enough to shoot.


    // TODO: Remove all the Limelight code. Getting the position will now be handled in a separate object.
     double[] limelight = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);


    // TODO: Delete this. We will determine the robot location in Positioning, not AutoAim.
    private double lastX;
    private double lastY;
    private double lastYaw;

    // TODO: Delete this. We will determine the robot location in Positioning, not AutoAim.
    //gets most previous x position
    public double getLastX() {
        lastX = limelight[0];
        return lastX;
    }

    // TODO: Delete this. We will determine the robot location in Positioning, not AutoAim.
    //get most previous y position
    public double getLastY(){
        lastY = limelight[1];
        return lastY;
    }

    // TODO: Delete this. We will determine the robot location in Positioning, not AutoAim.
    //gets most previous YAW position
        public double getLastYaw() {
        lastYaw = limelight[5];
        return lastYaw;
    }

    // TODO: Rename this to getDesiredYawInDegreesToSpeaker with arguments of X,Y, and isBlueAlliance.
    public double getDesiredYawInDegrees() {
       double initXPos = getLastX();
       double initYPos = getLastY();
       double desiredYaw = Math.atan(initYPos/initXPos);
        return desiredYaw;
    }

    // This function determines what angle we should turn to if we want to "shoot" into the Amp.
    public double getDesiredYawInDegreesToAmp(boolean isBlueAlliance) {
        // TODO: Implement this function.
        // Hint: It doesn't matter where our robot is. When we want to go to the Amp, we should just
        //       turn to that side of the field.
        return 0.0;
    }

    // This function determines what angle we should turn to if we want to "intake" from the Source.
    public double getDesiredYawInDegreesToSource(boolean isBlueAlliance) {
        // TODO: Implement this function. Remember, that our robot's back side intakes.
        // Hint: It doesn't matter where our robot is. When we want to go to the Source, we should just
        //       turn parallell to the source.
        return 0.0;
    }

    // This function returns if we are closer to the Amp than the speaker.
    // This function will be used to help determine what we want to do when the Driver says arm up.
    public boolean isAmpCloser(double x, double y) {
        // TODO: Implement this function. Calculate the distance to amp and speaker, then determine
        //       if amp is closer.
        return false;
    }

    // This function returns the correct direction to turn if the driver has the arm fully up.
    // When the arm is fully up, we are either intaking from the Source, or placing in the Amp.
    // This automatically switches the rotation of the robot based on the closest of the two.
    public boolean getDesiredYawInDegreesIfArmUp(double x, double y) {
        // TODO: Implement this function. Hint: Call the functions above 
        return false;
    }

    // TODO: Add X/Y arguments to this function so we can pass in the location from which ever Limelight sees an AprilTag. 
    // TODO: Change the argument to isBlueAlliance, that is the default for WPILIB see the link below.
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin
    public double getDesiredShooterAngleInDegrees(boolean isRedAlliance){
        if (isRedAlliance==true){
            double xDistance = Constants.SPEAKERXFROMCENTER - getLastX(); //calculates x distance from speaker in meters
            double yDistance = Constants.SPEAKERYFROMCENTER - getLastY(); //calculates y distance from speaker in meters
            double displacementFromSpeaker = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2)); //combines the x and y components to get our displacement from the speaker
            double desiredAngle = Math.atan(Constants.SPEAKERHEIGHT/displacementFromSpeaker); //finds our desired angle based on the height of the speaker and our displacement from the speaker
            return desiredAngle;
        } else {
            double xDistance = -Constants.SPEAKERXFROMCENTER - getLastX(); //calculates x distance from speaker in meters
            double yDistance = -Constants.SPEAKERYFROMCENTER - getLastY(); //calculates y distance from speaker in meters
            double displacementFromSpeaker = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2)); //combines the x and y components to get our displacement from the speaker
            double desiredAngle = Math.atan(Constants.SPEAKERHEIGHT/displacementFromSpeaker); //finds our desired angle based on the height of the speaker and our displacement from the speaker
            return desiredAngle;   
        }
    }

    // TODO: Change the argument to isBlueAlliance, that is the default for WPILIB see the link below.
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin
    public boolean legalToShoot(boolean isRedAlliance){ //returns true if we are not in opposing alliance's wing
        boolean isLegalToShoot;
        double currentXPos = getLastX();
        if(isRedAlliance == true && currentXPos <= -Constants.WINGDISTANCEFROMCENTER){//returns false if we are red alliance and in Blue ALliance's wing
            isLegalToShoot = false;
        } 
        else if (isRedAlliance == false && currentXPos >= Constants.WINGDISTANCEFROMCENTER){   //returns false if we are blue alliance and in Red Alliance's wing
            isLegalToShoot = false;
        }   
        else {
            isLegalToShoot = true;
        }
            return isLegalToShoot;
    }

    // This function determines whether we will hit the stage if we shoot now.
    // TODO: Add x/y arguments to this function.
    public boolean safeToShoot(boolean isRedAlliance){
        // TODO: Implement this function.
        // Hint: Think of a line drawn between our robot and the speaker.
        //       Think of the stage as a circle.
        //       Google ways to determine if a line intersects a circle.
        boolean isSafeToShoot = true;
        return isSafeToShoot;
    }

    // Returns the current distance from the speaker.
    // This can be used to help determine whether a shot is within range.
    public double shootingDistanceInMetersToSpeaker(double x, double y, boolean isRedAlliance) {
        // TODO: Implement this function, calculate the distance away from each speaker.
        return 1000.0;
    }

    // This function determines if we should "pull the trigger", our our robot is not ready.
    // x: X coordinates in Blue origin field.
    // y: Y coordinates in Blue origin field.
    // yawInDegres: Robot's current yaw.
    // armAngleInDegres: Position of arm.
    public boolean alignedEnoughToShoot(double x, double y, double yawInDegrees, double armAngleInDegrees, boolean isBlueAlliance) {
        double distanceInMeters = shootingDistanceInMetersToSpeaker(x, y, isBlueAlliance);
        
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
        double desiredYawInDegrees = getDesiredYawInDegrees();
        double yawError = Math.abs(desiredYawInDegrees - yawInDegrees);
        if (yawError < allowedYawErrorInDegrees) {
            // Close enought
            yawAligned = true;
        } else {
            yawAligned = false;
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
        double desiredArmAngleInDegrees = getDesiredShooterAngleInDegrees(isBlueAlliance);
        double pitchError = Math.abs(desiredArmAngleInDegrees - armAngleInDegrees);
        if (pitchError < allowedArmErrorInDegrees) {
            pitchAligned = true;
        } else {
            pitchAligned = false;
        }

        if (yawAligned && pitchAligned) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("AutoAim");
        builder.addDoubleProperty("desiredYaw", this::getDesiredYawInDegrees, null);
       // builder.addDoubleProperty("desiredShootAngle", this::getDesiredShooterAngleInDegrees, null);

        builder.addDoubleProperty("lastX", this::getLastX, null);
    }

}
