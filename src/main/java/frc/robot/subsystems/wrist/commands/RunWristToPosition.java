package frc.robot.subsystems.wrist.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class RunWristToPosition extends CommandBase{
    WristSubsystem wristSubsystem;
    double position;
    public RunWristToPosition(WristSubsystem wristSubsystem, double position) {
        this.wristSubsystem = wristSubsystem;
        addRequirements(wristSubsystem);
        this.position = position;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        wristSubsystem.runWristToPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        wristSubsystem.runWrist(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(position - wristSubsystem.getWristPosition()) < .03;
    }
}
