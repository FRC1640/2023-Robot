package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;

    
public class setCymbalTurnedCommand extends CommandBase{
    // ArmSubsystem armSubsystem;
    XboxController opController;
    GrabberSubsystem grabberSubsystem;
    boolean isTurned;
    
    public setCymbalTurnedCommand(GrabberSubsystem grabberSubsystem, boolean isTurned) {
        this.grabberSubsystem = grabberSubsystem;
        //this.opController = opController;
        this.isTurned = isTurned;
        addRequirements(grabberSubsystem);


    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        grabberSubsystem.setServoTurned(isTurned);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
