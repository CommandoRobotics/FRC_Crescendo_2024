package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;


public class AutoAim implements Sendable {

     double[] limelight = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);



    private double lastX;
    private double lastY;
    private double lastYaw;

    //gets most previous x position
    public double getLastX() {
        lastX = limelight[0];
        return lastX;
    }

    //get most previous y position
    public double getLastY(){
        lastY = limelight[1];
        return lastY;
    }
    //gets most previous YAW position
        public double getLastYaw() {
        lastYaw = limelight[5];
        return lastYaw;
    }

    // Hey Mason,
    // It looks like the getDoubleArray() grabs the current value of the Network Tables, so to make this work, I had to add
    // this update() function. You could either do this, or make this call as part of your getDesiredYawInDegrees.
    // -Mr. Barber
    public void update() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
    }

    // Hey Mason,
    // This is function is actually returning Radians, not degrees.
    // Also, this gets the desired yaw in relation to the origin (0,0) of the field, not the target.
    // You might want to add getDesiredYawRedSpeaker(), getDesiredRawBlueSpeaker(), etc... functions
    // that all might call a commmon getDesiredYaw(targetX, targetY).
    // One more thing, I think the fron of the robot is pointing the opposite direction from where it should be.
    // -Mr. Barber
    public double getDesiredYawInDegrees() {
       double initXPos = getLastX();
       double initYPos = getLastY();
       double desiredYaw = Math.atan(initYPos/initXPos);
        return desiredYaw;
    }

    // Hey Mason,
    // As we discussed, this still needs to be implemented.
    // -Mr. Barber
    public double getDesiredShooterAngleInDegrees(){
        return 0.0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("AutoAim");
        builder.addDoubleProperty("desiredYaw", this::getDesiredYawInDegrees, null);
        builder.addDoubleProperty("desiredShootAngle", this::getDesiredShooterAngleInDegrees, null);

        builder.addDoubleProperty("lastX", this::getLastX, null);
    }





}
