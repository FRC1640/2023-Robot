package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmManualCommand extends CommandBase{
    ArmSubsystem armSubsystem;
    XboxController opController;
    public ArmManualCommand(ArmSubsystem armSubsystem, XboxController opController) {
        this.opController = opController;
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armSubsystem.setSpeedLower(opController.getLeftY() * 0.3);
        armSubsystem.setSpeedUpper(opController.getRightY() * 0.3);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopArm();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
