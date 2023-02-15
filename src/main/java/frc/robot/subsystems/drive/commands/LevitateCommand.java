package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.DistanceSensor;
import frc.robot.subsystems.drive.DriveSubsystem;

public class LevitateCommand extends CommandBase {
  DigitalInput levitateSwitch = new DigitalInput(0);//TODO: add IDs
  DistanceSensor leftSensor = new DistanceSensor(4);//TODO: add IDs
  DistanceSensor rightSensor = new DistanceSensor(5);//TODO: add IDs
  DriveSubsystem driveSubsystem;

  public LevitateCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void initialize() {
  }

  @Override
    public void execute() {
      if (!leftSensor.isOff() && !rightSensor.isOff()){
        driveSubsystem.drive(0, 0.1, 0, false);
      }//drive right until both sensors are out of the platform
      if (leftSensor.isOff()){
        driveSubsystem.driveAxis(0, 0, 0.1, false, new Translation2d(-10.375,12.375));
      }//if left sensor is out of the platform, pivot on left wheel
      if (rightSensor.isOff()){
        driveSubsystem.driveAxis(0, 0, 0.1, false, new Translation2d(-10.375,-12.375));
      }//if right sensor is out of the platform, pivot on right wheel
      if (leftSensor.isOff() && rightSensor.isOff()){
        //TODO: PUT DOWN FOOT
        if (levitateSwitch.get()) {
          driveSubsystem.drive(0, 0.0, 0, false);
          driveSubsystem.xWheels();
        } else {
          driveSubsystem.drive(0, 0.1, 0, false);
        }//if limit switch is triggered, stop all motors
      }
    }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}