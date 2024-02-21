// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 0;
  }

  //All the following are in meters
  public static final double SPEAKERXFROMCENTER = 8.294;
  public static final double SPEAKERYFROMCENTER = 1.442; //this is the center of the speaker's distance from the center of the field, in reality the speaker is 1.53 meters wide
  public static final double SPEAKERHEIGHT = 1.989;
  public static final double WINGDISTANCEFROMCENTER = 2.43;

  // Limelight Positions
  public static final double kLimelight1MetersAboveField = 1.0; // TODO: Tune this. Distance above field to center of camera lens (meters).
  public static final double kLimelight1MetersFromRobotCenterX = 0.1; // TODO: Tune this. Left of center is positive.
  public static final double kLimelight1MetersFromRobotCenterY = 0.1; // TODO: Tune this. Toward front of center is positve.
  public static final double kLimelight1RotationInDegrees = 0.0; // TODO: Tune this. Rotation is COUNTER-clockwise from center of robot.
  public static final Pose2d kLimelight1Pose = new Pose2d(
    Constants.kLimelight1MetersFromRobotCenterX,
    Constants.kLimelight1MetersFromRobotCenterY,
    Rotation2d.fromDegrees(Constants.kLimelight1RotationInDegrees)
  );


  public static final double kLimelight2MetersAboveField = 1.0; // TODO: Tune this. Distance above field to center of camera lens (meters).
  public static final double kLimelight2MetersFromRobotCenterX = 0.1; // TODO: Tune this. Left of center is positive.
  public static final double kLimelight2MetersFromRobotCenterY = 0.1; // TODO: Tune this. Toward front of center is positve.
  public static final double kLimelight2RotationInDegrees = 0.0; // TODO: Tune this. Rotation is COUNTER-clockwise from center of robot.
  public static final Pose2d kLimelight2Pose = new Pose2d(
    Constants.kLimelight2MetersFromRobotCenterX,
    Constants.kLimelight2MetersFromRobotCenterY,
    Rotation2d.fromDegrees(Constants.kLimelight2RotationInDegrees)
  );
}
