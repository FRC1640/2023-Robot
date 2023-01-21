package frc.robot.auton.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Balance extends CommandBase{
    private final DriveSubsystem driveSubsystem;
    PIDController rotateController = new PIDController(0.0015, 0, 0.0);
    PIDController driveController = new PIDController(0, 0, 0);

    public Balance(DriveSubsystem swerve) {
  
      driveSubsystem = swerve;
  
      addRequirements(driveSubsystem);
  
    }
    @Override
    public void initialize() {
        driveController.setIntegratorRange(-0.4, 0.4);
    }
    @Override
    public void execute(){
        double rotateSetpoint = 0;
        double rot = rotateController.calculate(driveSubsystem.getGyroAngleDegrees(), rotateSetpoint);
        
        double drive = rotateController.calculate(driveSubsystem.getGyroPitch(), rotateSetpoint);
        driveSubsystem.drive(drive, 0, rot, false);
        
    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished() {
      return false;
    }
}
