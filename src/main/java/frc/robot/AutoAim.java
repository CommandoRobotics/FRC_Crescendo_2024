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

    public double getDesiredYawInDegrees() {
       double initXPos = getLastX();
       double initYPos = getLastY();
       double desiredYaw = Math.atan(initYPos/initXPos);
        return desiredYaw;
    }

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
