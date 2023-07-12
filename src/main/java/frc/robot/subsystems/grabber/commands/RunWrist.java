package frc.robot.subsystems.grabber.commands;

import org.ejml.equation.IntegerSequence.Combined;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class RunWrist extends CommandBase{
    GrabberSubsystem grabberSubsystem;
    double speed;
    public RunWrist(GrabberSubsystem grabberSubsystem, double speed) {
        this.grabberSubsystem = grabberSubsystem;
        addRequirements(grabberSubsystem);
        this.speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        grabberSubsystem.runWrist(speed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
