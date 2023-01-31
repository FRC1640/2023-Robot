package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utilities.Logger;

public class ResetGyro extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private Gyro gyro;
    public ResetGyro (DriveSubsystem driveSubsystem, Gyro gyro) {
        this.driveSubsystem = driveSubsystem;
        this.gyro = gyro;
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        gyro.resetGyro();
        gyro.setOffset(0);
        Logger.log("Gyro reset!");
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
