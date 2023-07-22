package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class GrabberSpin extends CommandBase{
    GrabberSubsystem grabberSubsystem;
    double speed;
    public GrabberSpin(GrabberSubsystem grabberSubsystem, double speed) {
        this.grabberSubsystem = grabberSubsystem;
        addRequirements(grabberSubsystem);
        this.speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        grabberSubsystem.spinGrabber(speed);
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
