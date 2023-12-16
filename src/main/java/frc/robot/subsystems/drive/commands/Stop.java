package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Stop extends Command {

    DriveSubsystem driveSubsystem;
    long startTime;
    
    public Stop(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // System.out.println("Point wheels");
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        driveSubsystem.xWheels();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false);
        //System.out.println("end");
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= startTime + 500;
    }



}
