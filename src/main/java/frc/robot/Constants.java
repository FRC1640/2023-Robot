// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static double HIGH_ANGLE = 5;
    public static double SERVO_WAIT = 0.5;
    public static double CYMBAL_SERVO_MID_ANGLE = 30;
    public static double CYMBAL_SERVO_UPRIGHT_ANGLE = 90; 
    // public static double CYMBAL_SERVO_GROUND_ANGLE  = 120; // UNNECESSARY because we dont need to spin if we do ground pickup
  }
}
