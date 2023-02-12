package frc.robot.auton.commands;

import java.util.Currency;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;

public class EndPitch extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    double initTime;
    boolean setTime = true;
    private Gyro gyro;
    public EndPitch(DriveSubsystem swerve, Gyro gyro) {
        this.gyro = gyro;
        driveSubsystem = swerve;
    
      }
      @Override
      public void initialize() {
      }
      @Override
      public void execute(){
        if (Math.abs(gyro.getGyroPitch()) >= 13 && setTime){
          initTime = System.currentTimeMillis();
          setTime = false;
        }
        // System.out.println("Pitch: " + gyro.getGyroPitch());
      }
      @Override
      public void end(boolean interrupted){
        System.out.println("Balance!");
      }
      @Override
      public boolean isFinished() {
        return System.currentTimeMillis() - initTime >= 500 && !setTime;
      }
}
