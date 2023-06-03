package frc.robot.auton.commands;

import java.util.Currency;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;

public class EndPitch2 extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    double initTime;
    boolean setTime = true;
    private Gyro gyro;
    boolean two = false;
    double initTime2;
    int delay2;
    public EndPitch2(DriveSubsystem swerve, Gyro gyro, int delay2) {
        this.gyro = gyro;
        driveSubsystem = swerve;
        this.delay2 = delay2;
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
        if (!setTime && System.currentTimeMillis() - initTime > delay2){
          if (!two && Math.abs(gyro.getGyroPitch()) >= 13){
            two = true;
            initTime2 = System.currentTimeMillis();
          }
        }
        // System.out.println("Pitch: " + gyro.getGyroPitch());
      }
      @Override
      public void end(boolean interrupted){
        System.out.println("Balance!");
      }
      @Override
      public boolean isFinished() {
        return two && System.currentTimeMillis() - initTime2 >= 500;
      }
}
