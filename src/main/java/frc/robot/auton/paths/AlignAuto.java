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
        swerve.resetOdometry(new Pose2d(new Translation2d(swerve.getBotPose().getX() + 8, swerve.getBotPose().getY() + 4), swerve.getBotPose().getRotation()));
        Translation2d pose = swerve.getPose().getTranslation();
        double angle = Math.atan2(1 - pose.getY() , 13.8 - pose.getX());
        PathPlannerTrajectory alignWithTag = PathPlanner.generatePath(
            new PathConstraints(0.05, 0.01),
            new PathPoint(pose, new Rotation2d(0)),
            new PathPoint(new Translation2d(13.6, 0.7), new Rotation2d(0))
          );
        PPSwerveControllerCommand path = new PPSwerveControllerCommand(alignWithTag,
        swerve::getPose, // Functional interface to feed supplier
        kDriveKinematics, new PIDController(0.3, 0.0, 0.0), new PIDController(0.3, 0.0, 0.0), new PIDController(0.5, 0, 0),
        swerve::setModuleStates, false, swerve);
        return path;
      }
}
