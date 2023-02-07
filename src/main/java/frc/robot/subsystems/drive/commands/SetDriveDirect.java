package frc.robot.subsystems.drive.commands;

import javax.sound.midi.ControllerEventListener;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;

public class SetDriveDirect extends CommandBase{
    DriveSubsystem driveSubsystem;
    XboxController controller;
    public SetDriveDirect (DriveSubsystem driveSubsystem, XboxController controller) {
        this.driveSubsystem = driveSubsystem;
        this.controller = controller;
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        System.out.println("Controller: " + controller.getLeftX());
        driveSubsystem.getMotor().set(controller.getLeftX());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
