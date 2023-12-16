package frc.robot.subsystems.drive.commands;

// import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.Gyro;
import frc.robot.utilities.Logger;

public class GyroOffsetCommand extends Command {

    private Gyro gyro;
    private double offset;
    public GyroOffsetCommand(Gyro gyro, double offset) {
        this.gyro = gyro;
        this.offset = offset;
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        gyro.resetGyro();
        gyro.setOffset(offset);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
