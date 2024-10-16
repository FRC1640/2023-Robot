package frc.robot.auton.paths.OldPaths;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auton.commands.Balance;
import frc.robot.auton.commands.EndPitch;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.Preset;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.GyroOffsetCommand;
import frc.robot.subsystems.drive.commands.ResetOdometryCommand;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.SetGrabCommand;
import frc.robot.subsystems.grabber.commands.UnGrab;

public class DriveTest {
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
  public static final double x = Units.inchesToMeters(10.375); // 10.375"
  public static final double y = Units.inchesToMeters(12.375); // 12.375"
  public static SwerveDriveKinematics kDriveKinematics;

  
  PathPlannerTrajectory placePath = PathPlanner.loadPath("test", new PathConstraints(2, 2));
  PathPlannerState placeState = new PathPlannerState();
  /** Example static factory for an autonomous command. */
  public CommandBase loadAuto(Gyro gyro, DriveSubsystem swerve, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) { 
    placeState = placePath.getInitialState();
    GyroOffsetCommand gyroCommand = new GyroOffsetCommand(gyro, 0);
    kDriveKinematics = swerve.createKinematics();
    Pose2d placePose = new Pose2d(placeState.poseMeters.getTranslation(), placeState.holonomicRotation);
    Command resetOdo = new ResetOdometryCommand(swerve, placePose);

    Command place = armSubsystem.createEndEffectorProfileCommandNoInstant(Preset.HighPlacing);
    SequentialCommandGroup placeWait = new SequentialCommandGroup(new WaitCommand(0.75), place, new WaitCommand(1.3));
    Command safe = armSubsystem.createEndEffectorProfileCommandNoInstant(Preset.Pickup);

    Command pickup = armSubsystem.createEndEffectorProfileCommandNoInstant(Preset.Pickup);

    

    Command grab = new SetGrabCommand(grabberSubsystem, true);
    ParallelDeadlineGroup grabGroup = new ParallelDeadlineGroup(placeWait, placeWait, grab);
    Command setConeMode = new InstantCommand(() -> armSubsystem.setIsInCubeMode(false));
    Command unGrab = new UnGrab(grabberSubsystem);

    PPSwerveControllerCommand placePathController = new PPSwerveControllerCommand(placePath,
        swerve::getPose, // Functional interface to feed supplier
        kDriveKinematics, new PIDController(0.05, 0.0, 0), new PIDController(1, 0.0, 0), new PIDController(0.4, 0, 0),
        swerve::setModuleStates, true, swerve);
    
    // ParallelCommandGroup group = new ParallelCommandGroup(safe, placePathController);
    // return Commands.sequence(resetOdo, group);
    return Commands.sequence(gyroCommand, resetOdo, placePathController);// , place, group
  }
}
