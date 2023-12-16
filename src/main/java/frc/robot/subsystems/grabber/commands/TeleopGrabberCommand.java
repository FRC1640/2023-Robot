package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class TeleopGrabberCommand extends Command {
    private GrabberSubsystem grabberSubsystem;
    public TeleopGrabberCommand(GrabberSubsystem grabberSubsystem){
        this.grabberSubsystem = grabberSubsystem;
        addRequirements(grabberSubsystem);
    }
    @Override
    public void execute() {
        grabberSubsystem.setClamped(true);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
    @Override 
    public void end(boolean interrupted){
        grabberSubsystem.setClamped(false);
    }


    
}
