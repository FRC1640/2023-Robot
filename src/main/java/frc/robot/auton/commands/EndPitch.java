package frc.robot.auton.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class EndPitch extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    public EndPitch(DriveSubsystem swerve) {
  
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
        return driveSubsystem.getGyroPitch() >= 6;
      }
}
