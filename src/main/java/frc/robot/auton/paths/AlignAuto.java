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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.sensors.Gyro;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AlignAuto {
    
  


    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    public static final double x = Units.inchesToMeters(10.375); // 10.375"
    public static final double y = Units.inchesToMeters(12.375); // 12.375"
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(y, x),new Translation2d(y, -x), new Translation2d(-y, x), new Translation2d(-y, -x));
    public CommandBase loadAuto(Gyro gyro, DriveSubsystem swerve, Limelight limelight) { 
        gyro.resetGyro();
        gyro.setOffset(-limelight.getBotPose().getRotation().getDegrees());
        
        
        swerve.resetOdometry(new Pose2d(new Translation2d(limelight.getBotPose().getX() + 8, limelight.getBotPose().getY() + 4), limelight.getBotPose().getRotation()));
        Translation2d pose = swerve.getPose().getTranslation();
        
        // System.out.println("Pose: " + pose);
        PathPlannerTrajectory alignWithTag = PathPlanner.generatePath(
            new PathConstraints(2, 1),
            new PathPoint(pose, new Rotation2d(0), new Rotation2d(swerve.getPose().getRotation().getRadians())),
            new PathPoint(new Translation2d(14.3, 1.3), new Rotation2d(0), new Rotation2d(0))
          );
        PPSwerveControllerCommand path = new PPSwerveControllerCommand(alignWithTag,
        swerve::getPose, // Functional interface to feed supplier
        kDriveKinematics, new PIDController(0.4, 0.0, 0.0), new PIDController(0.4, 0.0, 0.0), new PIDController(0.5, 0, 0),
        swerve::setModuleStates, false, swerve);
        return path;
      }
}
