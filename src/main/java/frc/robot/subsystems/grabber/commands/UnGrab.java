package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class UnGrab extends CommandBase{
    private GrabberSubsystem grabberSubsystem;
    public UnGrab(GrabberSubsystem grabberSubsystem){
        this.grabberSubsystem = grabberSubsystem;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        grabberSubsystem.setClamped(false);
    }

    @Override 
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
