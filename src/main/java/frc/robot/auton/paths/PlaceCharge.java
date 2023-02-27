package frc.robot.auton.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.auton.commands.Balance;
import frc.robot.auton.commands.EndPitch;
import frc.robot.auton.commands.EndPitch2;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.Preset;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.ResetOdometryCommand;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.SetGrabCommand;

public class PlaceCharge {
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
  public static final double x = Units.inchesToMeters(10.375); // 10.375"
  public static final double y = Units.inchesToMeters(12.375); // 12.375"
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(y, x),new Translation2d(y, -x), new Translation2d(-y, x), new Translation2d(-y, -x));

  
  PathPlannerTrajectory placePath = PathPlanner.loadPath("place - out - charge", new PathConstraints(2, 2));
  PathPlannerState placeState = new PathPlannerState();
  /** Example static factory for an autonomous command. */
  public CommandBase loadAuto(Gyro gyro, DriveSubsystem swerve, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) { 
    placeState = placePath.getInitialState();
    gyro.resetGyro();
    gyro.setOffset(180);
    Pose2d placePose = new Pose2d(placeState.poseMeters.getTranslation(), placeState.holonomicRotation);
    Command resetOdo = new ResetOdometryCommand(swerve, placePose);

    Command place = new InstantCommand(
        () -> armSubsystem.createEndEffectorProfileCommand(Preset.HighPlacing).schedule());

    Command safe = new InstantCommand(
        () -> armSubsystem.createEndEffectorProfileCommand(Preset.Pickup).schedule());

    Command pickup = new InstantCommand(
        () -> armSubsystem.createEndEffectorProfileCommand(Preset.Pickup).schedule());

    

    Command grab = new SetGrabCommand(grabberSubsystem, true);
    Command unGrab = new SetGrabCommand(grabberSubsystem, false);

    PPSwerveControllerCommand placePathController = new PPSwerveControllerCommand(placePath,
        swerve::getPose, // Functional interface to feed supplier
        kDriveKinematics, new PIDController(0.3, 0.0, 0.0), new PIDController(0.3, 0.0, 0.0), new PIDController(0.5, 0, 0),
        swerve::setModuleStates, true, swerve);
    
    EndPitch2 end = new EndPitch2(swerve, gyro);
    ParallelDeadlineGroup group = new ParallelDeadlineGroup(end, safe, placePathController, end);
    return Commands.sequence(resetOdo, pickup, grab, place, unGrab, group);
  }
}
