package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.DistanceSensor;
import frc.robot.subsystems.drive.DriveSubsystem;

public class LevitateCommand extends CommandBase{
    DistanceSensor leftSensor = new DistanceSensor(4);
    DistanceSensor rightSensor = new DistanceSensor(5);
    DriveSubsystem driveSubsystem;
    public LevitateCommand (DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
      if (!leftSensor.isOff() && !rightSensor.isOff()){
        driveSubsystem.drive(0, 0.1, 0, false);
      }
      if (leftSensor.isOff()){
        driveSubsystem.driveAxis(0, 0, 0.1, false, new Translation2d(-10.375,12.375));
      }
      if (rightSensor.isOff()){
        driveSubsystem.driveAxis(0, 0, 0.1, false, new Translation2d(-10.375,-12.375));
      }
      if (leftSensor.isOff() && rightSensor.isOff()){
        //drive right and foot
      }
      
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
    // while (!leftSensor.isOff() && !rightSensor.isOff()){ 
    //   //driveRightSlowly();
    // }

    // if (leftSensor.isOff()){
    //   while(!rightSensor.isOff()){
    //     //pivotOnLeftWheel();
    //   }
    // } 
    // if (rightSensor.isOff()){
    //   while(!leftSensor.isOff()){
    //     //pivotOnRightWheel();
    //   }
    // }
        //rampDriveOff();//deployes rubber foot and drives off in one function    
}