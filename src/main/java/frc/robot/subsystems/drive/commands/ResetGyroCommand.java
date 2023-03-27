package frc.robot.subsystems.drive.commands;

// import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Gyro;
import frc.robot.utilities.Logger;

public class ResetGyroCommand extends CommandBase {

    private Gyro gyro;
    public ResetGyroCommand (Gyro gyro) {
        this.gyro = gyro;
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        gyro.resetGyro();
        gyro.setOffset(0);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
