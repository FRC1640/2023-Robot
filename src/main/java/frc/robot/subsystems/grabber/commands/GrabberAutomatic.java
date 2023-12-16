package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class GrabberAutomatic extends Command{
    final double speedNormal = -1;
    final double speedGrabbed = -0.05;
    final double outputCurrentThreshold = 50;
    boolean grabbed;
    GrabberSubsystem grabberSubsystem;
    XboxController controller;
    public GrabberAutomatic(GrabberSubsystem grabberSubsystem, XboxController controller) {
        this.grabberSubsystem = grabberSubsystem;
        this.controller = controller;
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
            controller.setRumble(RumbleType.kBothRumble, 0.2);
            
        }
        else{
            grabberSubsystem.spinGrabber(speedNormal);
            controller.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.spinGrabber(0);
        controller.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

