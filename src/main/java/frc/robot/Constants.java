// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class PhysicalDimensions {
    public static final double kLowerArmLength = Units.inchesToMeters(39);
    public static final double kUpperArmLength = Units.inchesToMeters(35.4);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ServoSmasAngles {
    public static double HIGH_ANGLE = 180;
    public static double SERVO_WAIT = 0.5;
    public static double CYMBAL_SERVO_MID_ANGLE = 30;
    public static double CYMBAL_SERVO_UPRIGHT_ANGLE = 90; 
    // public static double CYMBAL_SERVO_GROUND_ANGLE  = 120; // UNNECESSARY because we dont need to spin if we do ground pickup
  }
  public static class FieldConstants{
    public static final double fieldLength = Units.inchesToMeters(651.25);

    //blue 1, blue 2, blue 3,...,red 1, red 2, red 3
    //blue and red 1 is the placement closest to the community wall. 
    public static final Translation2d[] placementPositions = {new Translation2d(1.6, 5.2), new Translation2d(1.6,4.5), 
        new Translation2d(1.6,3.839959), new Translation2d(1.6,3.4), new Translation2d(1.6,2.7), 
        new Translation2d(1.6, 2.147737), new Translation2d(1.6, 1.694095), new Translation2d(1.6, 1.015319),
        new Translation2d(1.6, 0.55)};

    public static final double fieldWidth = Units.inchesToMeters(315.5);
    public static final List<AprilTag> aprilTags = List.of(
      new AprilTag(
              1,
              new Pose3d(
                      Units.inchesToMeters(610.77),
                      Units.inchesToMeters(42.19),
                      Units.inchesToMeters(18.22),
                      new Rotation3d(0.0, 0.0, Math.PI))),
      new AprilTag(
              2,
              new Pose3d(
                      Units.inchesToMeters(610.77),
                      Units.inchesToMeters(108.19),
                      Units.inchesToMeters(18.22),
                      new Rotation3d(0.0, 0.0, Math.PI))),
      new AprilTag(
              3,
              new Pose3d(
                      Units.inchesToMeters(610.77),
                      Units.inchesToMeters(174.19),
                      Units.inchesToMeters(18.22),
                      new Rotation3d(0.0, 0.0, Math.PI))),
      new AprilTag(
              4,
              new Pose3d(
                      Units.inchesToMeters(636.96),
                      Units.inchesToMeters(265.74),
                      Units.inchesToMeters(27.38),
                      new Rotation3d(0.0, 0.0, Math.PI))),
      new AprilTag(
              5,
              new Pose3d(
                      Units.inchesToMeters(14.25),
                      Units.inchesToMeters(265.74),
                      Units.inchesToMeters(27.38),
                      new Rotation3d())),
      new AprilTag(
              6,
              new Pose3d(
                      Units.inchesToMeters(40.45),
                      Units.inchesToMeters(174.19),
                      Units.inchesToMeters(18.22),
                      new Rotation3d())),
      new AprilTag(
              7,
              new Pose3d(
                      Units.inchesToMeters(40.45),
                      Units.inchesToMeters(108.19),
                      Units.inchesToMeters(18.22),
                      new Rotation3d())),
      new AprilTag(
              8,
              new Pose3d(
                      Units.inchesToMeters(40.45),
                      Units.inchesToMeters(42.19),
                      Units.inchesToMeters(18.22),
                      new Rotation3d())));
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
    new AprilTagFieldLayout(aprilTags, fieldLength, fieldWidth);
  }
}
