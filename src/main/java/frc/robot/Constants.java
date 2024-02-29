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

  // Arm Constants - all angles in degrees, with zero meaning flat/down and 90 degrees arm straight up.
  public static class ArmConstants {
    public static final double kMinimumAllowedAngle = 0.0;
    public static final double kMaximumAllowedAngle = 90.0;
    public static final double kFloorIntakeAngle = 0; // TODO: Tune this angle. The angle when intaking notes from ground.
    public static final double kAmpAngle = 89; // TODO: Tune this angle. The angle when placing in the amp.
    public static final double kSourceAngle = 65; // TODO: Tune this angle. The angle when intaking from the source.
    public static final boolean kLeftArmReversed = false; // TODO: Check this. Encoder angle should increase as arm goes up, otherwise set this true.
    public static final boolean kRightArmReversed = false; // TODO: Check this. Encoder angle should increase as arm goes up, otherwise set this true.
    public static final double kLeftArmEncoderOffsetInRotations = 0.1686; // TODO: Tune this angle. This is the reading when it the arm is parallel to the floor.
    public static final double kRightArmEncoderOffsetInRotations = 0.8313; // TODO: Tune this angle. This is the reading when it the arm is parallel to the floor.
    public static final int kRioDIOPortLeftEncoder = 0;
    public static final int kRioDIOPortRightEncoder = 1;
    public static final int kRioDIOPortUpLimitSwitch = 2;
    public static final int kRioDIOPortDownLimitSwitch = 3;
  }

  // All the following are in meters using WPI Lib Blue as the Origin
  public class FieldElements {
    // Speaker
    public static final double kBlueSpeakerX = 0.0;
    public static final double kBlueSpeakerY = 5.55;
    public static final double kRedSpeakerX = 16.52;
    public static final double kRedSpeakerY = 5.55;
    public static final double kSpeakerHeight = 2.0; // Point where we are going to aim for.
    // Amp - Center of opening
    public static final double kBlueAmpX = 1.84;
    public static final double kBlueAmpY = 8.2;
    public static final double kRedAmpX = 14.7;
    public static final double kRedAmpY = 8.2;
    // Source
    public static final double kBlueSourceX = 15.079; // Using April tag 1.
    public static final double kBlueSourceY = 0.25; // Using April tag 1.
    public static final double kRedSourceX = 1.46; // Using April tag 10.
    public static final double kRedSourceY = 0.25; // Using April tag 10.
  }
  public class FieldLines {
    // Field Lines
    public static final double kCenterLineX = 8.25;
    public static final double kBlueWingLineX = 5.87;
    public static final double kRedWingLineX = 10.72;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
  }

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
