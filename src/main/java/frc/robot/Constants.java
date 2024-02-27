// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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


  // Arm Constants - all angles in degrees, with zero meaning flat/down and 90 degrees arm straight up.
  public static class ArmConstants {
    public static final double kFloorIntakeAngle = 0; // TODO: Tune this angle. The angle when intaking notes from ground.
    public static final double kAmpAngle = 89; // TODO: Tune this angle. The angle when placing in the amp.
    public static final double kSourceAngle = 65; // TODO: Tune this angle. The angle when intaking from the source.
    public static final boolean kLeftArmReversed = false; // TODO: Check this. Encoder angle should increase as arm goes up, otherwise set this true.
    public static final boolean kRightArmReversed = false; // TODO: Check this. Encoder angle should increase as arm goes up, otherwise set this true.
    public static final double kLeftArmEncoderOffsetInRotations = 0; // TODO: Tune this angle. This is the reading when it the arm is parallel to the floor.
    public static final double kRightArmEncoderOffsetInRotations = 0; // TODO: Tune this angle. This is the reading when it the arm is parallel to the floor.
}
}
