package frc.robot.subsystems.drive.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveToPosition {

    public static Command align(DriveSubsystem driveSubsystem, Pose2d position, Gyro gyro){
        
        SwerveDriveKinematics kDriveKinematics = driveSubsystem.createKinematics();
        
        PathPlannerTrajectory alignWithTag = PathPlanner.generatePath(
            new PathConstraints(2, 2),
            new PathPoint(driveSubsystem.getPose().getTranslation(), new Rotation2d(0), new Rotation2d(driveSubsystem.getPose().getRotation().getRadians())),
            new PathPoint(position.getTranslation(), new Rotation2d(0), position.getRotation())
          );
        PPSwerveControllerCommand path = new PPSwerveControllerCommand(alignWithTag,
            driveSubsystem::getPose, // Functional interface to feed supplier
            kDriveKinematics, new PIDController(1, 0.0, 0.0), new PIDController(1, 0.0, 0.0), new PIDController(0.4, 0, 0),
            driveSubsystem::setModuleStates, false, driveSubsystem);
        return path;
    }
}
