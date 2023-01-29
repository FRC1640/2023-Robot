package frc.robot.auton.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AlignAuto {
    private final DriveSubsystem swerve = RobotContainer.drive;
  


    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    public static final double x = 0.276225; 
    public static final double y = 0.301625; 
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(y, x),new Translation2d(y, -x), new Translation2d(-y, x), new Translation2d(-y, -x));
      public CommandBase loadAuto(){
        //TODO: reset gyro to odometry angle from apriltag or set an offset. robot must be perfectly straight for auto to work currently.
        swerve.resetGyro();
        swerve.resetOdometry(new Pose2d(new Translation2d(swerve.getBotPose().getX() + 8, swerve.getBotPose().getY() + 4), swerve.getBotPose().getRotation()));
        Translation2d pose = swerve.getPose().getTranslation();
        
        System.out.println("Pose: " + pose);
        double angle = Math.atan2(1 - pose.getY() , 13.8 - pose.getX());
        PathPlannerTrajectory alignWithTag = PathPlanner.generatePath(
            new PathConstraints(2, 1),
            new PathPoint(pose, new Rotation2d(0)),
            new PathPoint(new Translation2d(14, 1.1), new Rotation2d(0))
          );
        PPSwerveControllerCommand path = new PPSwerveControllerCommand(alignWithTag,
        swerve::getPose, // Functional interface to feed supplier
        kDriveKinematics, new PIDController(0.4, 0.0, 0.0), new PIDController(0.4, 0.0, 0.0), new PIDController(0.5, 0, 0),
        swerve::setModuleStates, false, swerve);
        return path;
      }
}
