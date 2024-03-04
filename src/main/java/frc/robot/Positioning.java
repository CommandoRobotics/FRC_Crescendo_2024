package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;

// Provides the position of the robot using Limelight/AprilTags.
// Positions are provided relative to the WPI Blue Origin.
// https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin
public class Positioning implements Sendable {
    
    // The following store the "tv" value. This indicates if Limelight sees a target.
    // A value of 1 means it sees a target, whereas 0 means it does not.
    private double targetValid1; // Stores the "tv" value from Limelight 1.
    private double targetValid2; // Stores the "tv" value from Limelight 2.
    
    // Create array to store botpose (robot location/direction) from the Limelight
    // We will use the blue coordinate system, as this is a common one used by WPILib
    // 
    // Limelight provides 6 numbers:
    // X: Meters along long edge of field. Positive is AWAY frobm blue driver station.
    // Y: Meters along short edge of field. Positive is to the LEFT.
    // Z: Meters above field. Remember, this is the camera's position.
    // Roll: How left/right tilted the robot is (rarely used).
    // Pitch: How forward/backwards tilted the robot is (rarely used).
    // Yaw: Rotation (left/right), positive is COUNTER-clockwise.
    final int xIndex = 0;
    final int yIndex = 1;
    final int zIndex = 2;
    final int rollIndex = 3;
    final int pitchIndex = 4;
    final int yawIndex = 5;
    private double[] poseArrayCamera1; // Robot pose from Primary Limelight
    private double[] poseArrayCamera2; // Robot pose form Back-up Limelight

    // The following values will store the the determined values from this code.
    private boolean lastValid; // Whether we had a valid tag to determine location.
    private Pose2d lastPose; // Pose of the robot.
    private Pose2d lastCameraPose2d; // For debugging
    private double lastCameraZ; // For debugging
    private double lastID; // April tag to determine this data.

    // The following stores which April Tag the limelight is using for calculations
    private double biggestTag1;
    private double biggestTag2;

    Positioning() {
        lastPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        poseArrayCamera1 = new double[6];
        poseArrayCamera2 = new double[6];
        lastCameraPose2d = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    }

    // Returns whether positioning sees a valid target.
    // Call this before using any of the other calls.
    // If this returns false, the other data may be old/wrong.
    public boolean acquired() {
        return lastValid;
    }

    // Returns the location and direction of the robot according to WPI Blue origin.
    // Units in Pose2d are Meters.
    public Pose2d getPose() {
        return lastPose;
    }

    //gets most previous x position
    public double getX() {
        return lastPose.getX();
    }

    //get most previous y position
    public double getY(){
        return lastPose.getY();
    }

    //gets most previous YAW position
    public double getYaw() {
        return lastPose.getRotation().getDegrees();
    }

    // This needs to be called periodically so the robot location is updated.
    public void update() {
        // Network tables associated with each Limelight
        NetworkTable limelightTable1 = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTable limelightTable2 = NetworkTableInstance.getDefault().getTable("limeligh-backup");

        // Get whether the Limelights see valid data.
        final double default_value = -1; // This value is returned if the Limelight is not connected.
        targetValid1 = limelightTable1.getEntry("tv").getDouble(default_value);
        targetValid2 = limelightTable2.getEntry("tv").getDouble(default_value);

        // Get which April Tag ID the limelight is using to calculate values.
        biggestTag1 = limelightTable1.getEntry("tid").getDouble(default_value);
        biggestTag2 = limelightTable2.getEntry("tid").getDouble(default_value);

        // Get calculated botpose
        poseArrayCamera1 = limelightTable1.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        poseArrayCamera2 = limelightTable2.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

        // Get the data from whichever Limelight has valid data
        if (trustLimelight1()) {
            // Limelight 1 saw valid data, use its values.
            lastValid = true;
            lastCameraPose2d = parsePose(poseArrayCamera1);
            lastPose = translateToRobotPose(lastCameraPose2d, Constants.kLimelight1Pose);
            lastID = biggestTag1;
            lastCameraZ = poseArrayCamera1[zIndex];
        } else if (trustLimelight2()) {
            // Limelight 2 saw valid data (but Limelight 1 did not). Use this data.
           // Added variables for second camera's data.
            lastValid = true;
            lastCameraPose2d = parsePose(poseArrayCamera2);
            lastPose = translateToRobotPose(lastCameraPose2d, Constants.kLimelight2Pose);
            lastID = biggestTag2;
            lastCameraZ = poseArrayCamera2[zIndex];
        } else {
            // Neither camera has valid data.
            // Retain the last known data, but mark the data as invalid.
            lastValid = false;
        }
    }

    // Returns whether or not to trust the Limelight data.
    // tv: The Target Value from the Limelight
    // tid: ID value of the April tag it sees.
    // z: Height the camera thinks it is above the field.
    private boolean limelightHasValidPose(double tv, double tid, double z) {
        // If the tv value is less than one, we have invalid data
        if (tv < 1) {
            return false;
        }

        // This year's game uses will accept April tags 1-16. Anything else is wrong.
        if (tid < 1) {
            return false;
        } else if (tid > 16) {
            return false;
        }

        // When the Limelight can't compute a pose, it uses zeros for all locations.
        // Since the Z should be the height of the camera, a value of 0 means either
        // The Limelight data is wrong, or our camera broke off and is on the floor.
        if (z == 0) {
            return false;
        }

        // If nothing above indicated bad data, then we will trust it.
        return true;
    }

    private boolean trustLimelight1() {
        return limelightHasValidPose(targetValid1, biggestTag1, poseArrayCamera1[zIndex]);
    }

    private boolean trustLimelight2() {
        return limelightHasValidPose(targetValid2, biggestTag2, poseArrayCamera2[zIndex]);
    }


    // This gets Pose (location/rotation) data from the Limelight's BotPose Array
    // and returns it as a WPI Pose2D object.
    private Pose2d parsePose(double[] botposeArray) {
        double x = botposeArray[xIndex];
        double y = botposeArray[yIndex];
        double r = botposeArray[yawIndex];
        return new Pose2d(x, y, Rotation2d.fromDegrees(r));
    }

    // This adjusts the pose
    private Pose2d translateToRobotPose(Pose2d cameraDataPose, Pose2d cameraRelativePose) {
        double adjustedX = cameraDataPose.getX() + cameraRelativePose.getX();
        double adjustedY = cameraDataPose.getY() + cameraRelativePose.getY();
        Rotation2d adjustedRotation = cameraDataPose.getRotation().plus(cameraRelativePose.getRotation());
        return new Pose2d(adjustedX, adjustedY, adjustedRotation);
    }

    // This function updates the Smart Dashboard with variables so we can debug.
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Positioning");
        builder.addDoubleProperty("lastX", this::getX, null);
        builder.addDoubleProperty("lastY", this::getY, null);
        builder.addDoubleProperty("lastZ", this::dashboardGetZ, null);
        builder.addDoubleProperty("lastYaw", this::getYaw, null);
        builder.addDoubleProperty("lastID", this::dashboardGetAprilTagID, null);
        builder.addBooleanProperty("trust1", this::trustLimelight1, null);
        builder.addBooleanProperty("trust2", this::trustLimelight2, null);
        builder.addDoubleProperty("camera1X", this::dashboardGetLimelight1X, null);
        builder.addDoubleProperty("camera1Y", this::dashboardGetLimelight1Y, null);
        builder.addDoubleProperty("camera1Yaw", this::dashboardGetLimelight1Yaw, null);
        builder.addDoubleProperty("camera2X", this::dashboardGetLimelight2X, null);
        builder.addDoubleProperty("camera2Y", this::dashboardGetLimelight2Y, null);
        builder.addDoubleProperty("camera2Yaw", this::dashboardGetLimelight2Yaw, null);
    }

    // Return the current April Tag ID we are using.
    private double dashboardGetAprilTagID() {
        return lastID;
    }

    private double dashboardGetZ() {
        return lastCameraZ;
    }

    // Return the position that Limelight1 thinks we are at.
    private double dashboardGetLimelight1X() {
        return translateToRobotPose(parsePose(poseArrayCamera1), Constants.kLimelight1Pose).getX();
    }
    private double dashboardGetLimelight1Y() {
        return translateToRobotPose(parsePose(poseArrayCamera1), Constants.kLimelight1Pose).getX();
    }
    private double dashboardGetLimelight1Yaw() {
        return translateToRobotPose(parsePose(poseArrayCamera1), Constants.kLimelight1Pose).getRotation().getDegrees();
    }

    private double dashboardGetLimelight2X() {
        return translateToRobotPose(parsePose(poseArrayCamera2), Constants.kLimelight2Pose).getX();
    }
    private double dashboardGetLimelight2Y() {
        return translateToRobotPose(parsePose(poseArrayCamera2), Constants.kLimelight2Pose).getX();
    }
    private double dashboardGetLimelight2Yaw() {
        return translateToRobotPose(parsePose(poseArrayCamera2), Constants.kLimelight2Pose).getRotation().getDegrees();
    }

    // Call this to force simulated values for X, Y, and Yaw.
    public void simulationPeriodic(double x, double y, double yaw) {
        NetworkTable limelightTable1 = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTable limelightTable2 = NetworkTableInstance.getDefault().getTable("limeligh-backup");

        final double simulateAcquired = 1;
        limelightTable1.getEntry("tv").setDouble(simulateAcquired);
        limelightTable2.getEntry("tv").setDouble(simulateAcquired);

        final double simulateAprilTagID = 1;
        limelightTable1.getEntry("tid").setDouble(simulateAprilTagID);
        limelightTable2.getEntry("tid").setDouble(simulateAprilTagID);

        final double poseValues[] = {x, y, 0.5, 0, 0, yaw};
        limelightTable1.getEntry("botpose_wpiblue").setDoubleArray(poseValues);
        limelightTable2.getEntry("botpose_wpiblue").setDoubleArray(poseValues);
    }

}
