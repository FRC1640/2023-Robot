package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

public class SetCubeModeCommand extends Command {
    
    public SetCubeModeCommand(ArmSubsystem armSubsystem, boolean isInCubeMode) {
        addRequirements(armSubsystem);
    }

}
