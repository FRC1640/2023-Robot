package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ArrayToPose {
    
    public static Pose3d convert(double[] poseArray) {
        if (poseArray.length < 6) {
            throw new IllegalArgumentException("Need at least 6 parameters for pose");
        }

        Translation3d translation = new Translation3d((poseArray[0]), poseArray[1], poseArray[2]);
        Rotation3d rotation = new Rotation3d(poseArray[3], poseArray[4], poseArray[5]);
        return new Pose3d(translation, rotation);
    }

}
