package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class TeleopGrabberCommand extends CommandBase {
    private GrabberSubsystem grabberSubsystem;
    private XboxController opController;

    public TeleopGrabberCommand(GrabberSubsystem grabberSubsystem, XboxController opController){
        this.grabberSubsystem = grabberSubsystem;
        this.opController = opController;
        addRequirements(grabberSubsystem);
    }

    public void execute() {
        grabberSubsystem.setClamped(opController.getAButton());
    }

    public boolean isFinished(){
        return false;
    }


    
}
