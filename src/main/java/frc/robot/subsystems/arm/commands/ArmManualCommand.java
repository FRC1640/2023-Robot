package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ArmMode;

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
        armSubsystem.setMode(ArmMode.Manual);
    }

    @Override
    public void execute() {
        armSubsystem.setManualLower(opController.getLeftY() * 0.2);
        armSubsystem.setManualUpper(opController.getRightY() * 0.2);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setManualLower(0);
        armSubsystem.setManualUpper(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
