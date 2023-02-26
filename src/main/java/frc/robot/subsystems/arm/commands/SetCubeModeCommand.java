package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class SetCubeModeCommand extends CommandBase {
    
    public SetCubeModeCommand(ArmSubsystem armSubsystem, boolean isInCubeMode) {
        addRequirements(armSubsystem);
    }

}
