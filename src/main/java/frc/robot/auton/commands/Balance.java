package frc.robot.auton.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Balance extends CommandBase{
    private final DriveSubsystem driveSubsystem;
    PIDController rotateController = new PIDController(0.0015, 0, 0.0);
    PIDController driveController = new PIDController(0.0085, 0, 0.002);

    public Balance(DriveSubsystem swerve) {
  
      driveSubsystem = swerve;
  
      addRequirements(driveSubsystem);
  
    }
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute(){
        double rotateSetpoint = 0;
        //double rot = rotateController.calculate(driveSubsystem.getGyroAngleDegrees(), rotateSetpoint);
        
        //SUCCESS: increasing clamp, p, and clamp range
        double drive = 0;
        if (driveSubsystem.getGyroPitch() >= 3 || driveSubsystem.getGyroPitch() <= -3){
            drive = driveController.calculate(driveSubsystem.getGyroPitch(), rotateSetpoint);
        }
        else{
            drive = MathUtil.clamp(drive, -0.06, 0.06);
            // if (drive <= 0.01){
            //     drive = 0;
            //     driveSubsystem.xWheels();
            // }
        }
        driveSubsystem.drive(drive, 0, 0, false);

        System.out.println("Pitch: " + driveSubsystem.getGyroPitch());
        System.out.println("Drive: " + drive);
        
    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished() {
      return false;
    }
}
