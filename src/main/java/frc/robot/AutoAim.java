package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class AutoAim extends SubsystemBase {

     double[] limelight = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
     double isValidTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("<tv>").getDouble(0);




    private double lastX;
    private double lastY;
    private double lastYaw;
    public boolean isLegalToShoot;
    public boolean isRedAlliance;



    public boolean detectsValidTargets(){//returns true if limelight detects april tag+
        boolean seesTarget;
        if (isValidTarget == 0){
            seesTarget = true;
        } else {
            seesTarget  = false; 
        }
        return seesTarget;
    }

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

    public double getDesiredShooterAngleInDegrees(boolean isRedAlliance){
        if (isRedAlliance==true){
            double xDistance = Constants.SPEAKERXFROMCENTER - getLastX(); //calculates x distance from speaker in meters
            double yDistance = Constants.SPEAKERYFROMCENTER - getLastY(); //calculates y distance from speaker in meters
            double displacementFromSpeaker = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2)); //combines the x and y components to get our displacement from the speaker
            double desiredAngle = Math.atan(Constants.SPEAKERHEIGHT/displacementFromSpeaker); //finds our desired angle based on the height of the speaker and our displacement from the speaker
            return desiredAngle;
        } else {
            double xDistance = -Constants.SPEAKERXFROMCENTER - getLastX(); //calculates x distance from speaker in meters
            double yDistance =  Constants.SPEAKERYFROMCENTER - getLastY(); //calculates y distance from speaker in meters
            double displacementFromSpeaker = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2)); //combines the x and y components to get our displacement from the speaker
            double desiredAngle = Math.atan(Constants.SPEAKERHEIGHT/displacementFromSpeaker); //finds our desired angle based on the height of the speaker and our displacement from the speaker
            return desiredAngle;   
        }
    }

    //decides whether or not it is practical to go for the amp or for the source and then it angles accordingly, taking into account which alliance its on
    public double getDesiredAngleAmpOrSourceInDegrees(boolean goForAmp, boolean isRedAlliance){
        if (goForAmp==true && isRedAlliance ==true){
            double xDistanceFromAmp = Constants.AMPXFROMCENTER - getLastX(); //calculates x distance from Amp in meters
            double yDistanceFromAmp = Constants.AMPYFROMCENTER - getLastY(); //calculates y distance from Amp in meters
            double displacementFromAmp = Math.sqrt(Math.pow(xDistanceFromAmp, 2) + Math.pow(yDistanceFromAmp, 2)); //combines the x and y components to get our displacement from the Amp
            double desiredAmpAngle = Math.atan(Constants.SPEAKERHEIGHT/displacementFromAmp); //finds our desired angle based on the height of the Amp  and our displacement from the Amp
            return desiredAmpAngle;
        } else if (goForAmp==true && isRedAlliance == false) {
            double xDistanceFromAmp = -Constants.AMPXFROMCENTER - getLastX(); //calculates x distance from Amp in meters
            double yDistanceFromAmp = Constants.AMPYFROMCENTER - getLastY(); //calculates y distance from Amp in meters
            double displacementFromAmp = Math.sqrt(Math.pow(xDistanceFromAmp, 2) + Math.pow(yDistanceFromAmp, 2)); //combines the x and y components to get our displacement from the Amp
            double desiredAmpAngle = Math.atan(Constants.SPEAKERHEIGHT/displacementFromAmp); //finds our desired angle based on the height of the Amp and our displacement from the Amp
            return desiredAmpAngle;   
        } else if (goForAmp==false && isRedAlliance == true) {
            double xDistanceFromSource = Constants.SOURCEXFROMCENTER - getLastX(); //calculates x distance from Source in meters
            double yDistanceFromSource = -Constants.SOURCEYFROMCENTER - getLastY(); //calculates y distance from source in meters
            double displacementFromSource = Math.sqrt(Math.pow(xDistanceFromSource, 2) + Math.pow(yDistanceFromSource, 2)); //combines the x and y components to get our displacement from the source
            double desiredSourceAngle = Math.atan(Constants.SOURCEHEIGHT/displacementFromSource); //finds our desired angle based on the height of the source and our displacement from the source
            return desiredSourceAngle;
        }
          else{
            double xDistanceFromSource = -Constants.SOURCEXFROMCENTER - getLastX(); //calulates x distance from source in meters
            double yDistanceFromSource = -Constants.SOURCEYFROMCENTER - getLastY(); //calculates y distance from source in meters
            double displacementFromSource = Math.sqrt(Math.pow(xDistanceFromSource, 2) + Math.pow(yDistanceFromSource, 2)); //combines the x and y components to get our displacement from the source
            double desiredSourceAngle = Math.atan(Constants.SOURCEHEIGHT/displacementFromSource);  //finds our desired angle based on the height of the source and our displacement from the source
            return desiredSourceAngle;
        }
    }

    public boolean goForAmp(){ //returns true if u are in ur own wing
        boolean goForAmp;
        if (isLegalToShoot == false) {
            goForAmp = false;
        }
        else {
            goForAmp= true;
        }
        return goForAmp;
    } 

    

    public boolean legalToShoot(boolean isRedAlliance){ //returns true if we are not in opposing alliance's wing
        // boolean isLegalToShoot;
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

    public boolean safeToShoot(boolean isRedAlliance){
        if (isRedAlliance == true && getLastX() > Constants.SHALLOWBOUNDARY) {
            boolean isSafeToShoot = false;
            return isSafeToShoot;
        } else if (isRedAlliance == false && getLastX() < -Constants.SHALLOWBOUNDARY) {
            boolean isSafeToShoot = false;
            return isSafeToShoot;
        } else if (getLastX() < Constants.STAGEBLOCKLEFTBOUNDARY && getLastX() > Constants.STAGEBLOCKRIGHTBOUNDARY && getLastY() < Constants.STAGEBLOCKUPPERBOUNDARY && getLastY() > Constants.STAGEBLOCKLOWERBOUNDARY) {
            boolean isSafeToShoot = true;
            return isSafeToShoot;
        } else {
            boolean isSafeToShoot = true;
            return isSafeToShoot;
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
