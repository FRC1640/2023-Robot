package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utilities.Logger;

public class ResetOdometry extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private Pose2d pose;

    public ResetOdometry(DriveSubsystem driveSubsystem, Pose2d pose) {
        this.driveSubsystem = driveSubsystem;
        this.pose = pose;
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        driveSubsystem.resetOdometry(pose);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }

}
