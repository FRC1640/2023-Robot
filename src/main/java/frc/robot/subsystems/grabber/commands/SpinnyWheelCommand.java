package frc.robot.subsystems.grabber.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class SpinnyWheelCommand extends CommandBase{
    
    private boolean rollIn;

    public SpinnyWheelCommand(GrabberSubsystem grabberSubsystem, boolean rollIn){
        this.rollIn = rollIn;
    }
    
    @Override
    public void initialize(){}

    @Override
    public void execute() {}

    @Override 
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}