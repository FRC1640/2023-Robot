package frc.robot.subsystems.wrist.commands;

import org.ejml.equation.IntegerSequence.Combined;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class RunWrist extends CommandBase{
    WristSubsystem wristSubsystem;
    double speed;
    public RunWrist(WristSubsystem wristSubsystem, double speed) {
        this.wristSubsystem = wristSubsystem;
        addRequirements(wristSubsystem);
        this.speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        wristSubsystem.runWrist(speed);
    }

    @Override
    public void end(boolean interrupted) {
        wristSubsystem.runWrist(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
