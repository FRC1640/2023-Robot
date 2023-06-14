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
import frc.robot.Constants;
import frc.robot.auton.commands.Balance;
import frc.robot.auton.commands.EndPitch;
import frc.robot.auton.commands.EndPitch2;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.Preset;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.GyroOffsetCommand;
import frc.robot.subsystems.drive.commands.ResetOdometryCommand;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.SetGrabCommand;
import frc.robot.subsystems.grabber.commands.UnGrab;

public class RedPlaceCharge2 {
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
  public static final double x = Units.inchesToMeters(10.375); // 10.375"
  public static final double y = Units.inchesToMeters(12.375); // 12.375"
  public static SwerveDriveKinematics kDriveKinematics;

  PathPlannerTrajectory chargeForward = PathPlanner.loadPath("redSide - charge forward", new PathConstraints(1.5, 2));
  
  PathPlannerTrajectory chargeBackward = PathPlanner.loadPath("redSide - charge backward", new PathConstraints(2, 2));
  PathPlannerState placeState = new PathPlannerState();
  /** Example static factory for an autonomous command. */
  public CommandBase loadAuto(Gyro gyro, DriveSubsystem swerve, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) { 
    placeState = chargeForward.getInitialState();
    GyroOffsetCommand gyroCommand = new GyroOffsetCommand(gyro, 180);
    kDriveKinematics = swerve.createKinematics();
    Pose2d placePose = new Pose2d(placeState.poseMeters.getTranslation(), placeState.holonomicRotation);
    Command resetOdo = new ResetOdometryCommand(swerve, placePose);

    Command place =  armSubsystem.create2dEndEffectorProfileCommandNoInstant(Preset.HighPlacing,1.9, 4.3, 0.6, 2);
    SequentialCommandGroup placeWait = new SequentialCommandGroup(place.alongWith(new SequentialCommandGroup(new WaitCommand(Constants.ServoSmasAngles.SERVO_WAIT)/*,new InstantCommand(() -> grabberSubsystem.servoMove(Constants.ServoSmasAngles.HIGH_ANGLE))*/)));
    Command safe = armSubsystem.createEndEffectorProfileCommandNoInstant(Preset.LowPlacing);
    Command safe1 = armSubsystem.createEndEffectorProfileCommandNoInstant(Preset.LowPlacing);

    Command pickup = armSubsystem.createEndEffectorProfileCommandNoInstant(Preset.Pickup);

    
    WaitCommand wait = new WaitCommand(0.4);
    Command grab = new SetGrabCommand(grabberSubsystem, true);
    Command grabEnd = new ParallelDeadlineGroup(wait, grab, wait);
    Command setConeMode = new InstantCommand(() -> armSubsystem.setIsInCubeMode(false));
    Command unGrab = new UnGrab(grabberSubsystem);
    Command uprightCone = armSubsystem.createEndEffectorProfileCommandNoInstant(Preset.UprightConeGround);

    PPSwerveControllerCommand chargeForwardController = new PPSwerveControllerCommand(chargeForward,
        swerve::getPose, // Functional interface to feed supplier
        kDriveKinematics,  new PIDController(0.05, 0.0, 0), new PIDController(1, 0.0, 0), new PIDController(0.4, 0, 0),
        swerve::setModuleStates, false, swerve);

    PPSwerveControllerCommand chargeBackwardController = new PPSwerveControllerCommand(chargeBackward,
        swerve::getPose, // Functional interface to feed supplier
        kDriveKinematics, new PIDController(0.05, 0.0, 0), new PIDController(1, 0.0, 0), new PIDController(0.4, 0, 0),
        swerve::setModuleStates, false, swerve);
    
    ParallelCommandGroup group = new ParallelCommandGroup(uprightCone, chargeForwardController);
    ParallelCommandGroup group1 = new ParallelCommandGroup(safe1, chargeBackwardController);
    // return Commands.sequence(resetOdo, group);
    return Commands.sequence(gyroCommand, resetOdo, setConeMode, 
    placeWait, unGrab, group.andThen(new InstantCommand(() -> swerve.drive(0, 0, 0, false))), grabEnd,
     new EndPitch(swerve, gyro).deadlineWith(group1), new Balance(swerve, gyro));// , place, group
  }
}