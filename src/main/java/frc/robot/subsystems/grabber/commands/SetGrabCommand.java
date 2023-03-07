package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class SetGrabCommand extends CommandBase{
    private GrabberSubsystem grabberSubsystem;
    boolean set;
    double initTime;
    public SetGrabCommand(GrabberSubsystem grabberSubsystem, boolean set){
        this.grabberSubsystem = grabberSubsystem;
        this.set = set;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize(){
        initTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        grabberSubsystem.setClamped(set);
    }

    @Override 
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
