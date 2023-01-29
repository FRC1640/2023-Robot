package frc.robot.auton.commands;

import java.util.Currency;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class EndPitch extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    double initTime;
    boolean setTime = true;
    public EndPitch(DriveSubsystem swerve) {
  
        driveSubsystem = swerve;
    
      }
      @Override
      public void initialize() {
      }
      @Override
      public void execute(){
        if (Math.abs(driveSubsystem.getGyroPitch()) >= 13 && setTime){
          initTime = System.currentTimeMillis();
          setTime = false;
        }
        System.out.println("Pitch: " + driveSubsystem.getGyroPitch());
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
