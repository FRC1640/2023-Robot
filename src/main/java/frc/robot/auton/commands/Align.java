package frc.robot.auton.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Align extends CommandBase{
    private final DriveSubsystem driveSubsystem;
    PIDController driveController = new PIDController(0, 0, 0);

    public Align(DriveSubsystem swerve) {
  
      driveSubsystem = swerve;
  
      addRequirements(driveSubsystem);
  
    }
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute(){

    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished() {
      return false;
    }
}
