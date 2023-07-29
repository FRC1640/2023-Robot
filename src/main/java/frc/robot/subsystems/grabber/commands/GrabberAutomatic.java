package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class GrabberAutomatic extends CommandBase{
    final double speedNormal = -0.5;
    final double speedGrabbed = -0.05;
    final double outputCurrentThreshold = 45;
    boolean grabbed;
    GrabberSubsystem grabberSubsystem;
    public GrabberAutomatic(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        grabbed = false;
    }

    @Override
    public void execute() {
        if (grabberSubsystem.getRollerCurrentAverage() >= outputCurrentThreshold){
            grabbed = true;
        }
        
        if (grabbed){
            grabberSubsystem.spinGrabber(speedGrabbed);
        }
        else{
            grabberSubsystem.spinGrabber(speedNormal);
        }
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.spinGrabber(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

