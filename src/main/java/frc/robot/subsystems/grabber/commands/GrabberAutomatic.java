package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class GrabberAutomatic extends CommandBase{
    final double speedNormal = 0.3;
    final double speedGrabbed = 0.5;
    final double outputCurrentThreshold = 0;
    GrabberSubsystem grabberSubsystem;
    public GrabberAutomatic(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double speed = speedNormal;
        if (grabberSubsystem.getRollerCurrent() >= outputCurrentThreshold){
            speed = speedGrabbed;
        }
        grabberSubsystem.spinGrabber(speed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

