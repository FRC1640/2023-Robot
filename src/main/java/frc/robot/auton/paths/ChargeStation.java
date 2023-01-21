// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.paths;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auton.commands.Balance;
import frc.robot.auton.commands.EndPitch;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ChargeStation {
  private final DriveSubsystem swerve = RobotContainer.drive;
  


  public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
  public static final double x = 0.276225; 
  public static final double y = 0.301625; 
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(y, x),new Translation2d(y, -x), new Translation2d(-y, x), new Translation2d(-y, -x));


  PathPlannerTrajectory chargePath = PathPlanner.loadPath("Charge Station", new PathConstraints(3, 2));
  PathPlannerState chargeState = new PathPlannerState();
  /** Example static factory for an autonomous command. */
  public CommandBase loadAuto() { 
    chargeState = chargePath.getInitialState();
    Pose2d chargePose = new Pose2d(chargeState.poseMeters.getTranslation(), chargeState.holonomicRotation);



    PPSwerveControllerCommand chargePathController = new PPSwerveControllerCommand(chargePath,
    swerve::getPose, // Functional interface to feed supplier
    kDriveKinematics, new PIDController(0.3, 0.0, 0.0), new PIDController(0.3, 0.0, 0.0), new PIDController(0.5, 0, 0),
    swerve::setModuleStates, false, swerve);
    return Commands.sequence(new EndPitch(swerve).deadlineWith(chargePathController), new Balance(swerve));
  }

  public ChargeStation() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
