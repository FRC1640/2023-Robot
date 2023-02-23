package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.Preset;

public class ArmProfileWrapper {
    public Command lowerCommand(double lowerPos, ArmSubsystem armSubsystem){
        return new TrapezoidProfileCommand(new TrapezoidProfile(
  
        // Limit the max acceleration and velocity

        new TrapezoidProfile.Constraints(

            armSubsystem.getLowerArmMaxSpeed(),
            armSubsystem.getLowerArmMaxAccel()),

        // End at desired position in meters; implicitly starts at 0

        new TrapezoidProfile.State(lowerPos, 0)),

    // Pipe the profile state to the drive

    setpointState -> armSubsystem.setLowerVoltage(armSubsystem.getLowerFFVoltage(-setpointState.velocity))

    );
    }
    public Command upperCommand(double upperPos, ArmSubsystem armSubsystem){
        return new TrapezoidProfileCommand(new TrapezoidProfile(
  
        // Limit the max acceleration and velocity

        new TrapezoidProfile.Constraints(

            armSubsystem.getUpperArmMaxSpeed(),
            armSubsystem.getUpperArmMaxAccel()),

        // End at desired position in meters; implicitly starts at 0

        new TrapezoidProfile.State(upperPos, 0)),

    // Pipe the profile state to the drive

    setpointState -> armSubsystem.setUpperVoltage(armSubsystem.getUpperFFVoltage(-setpointState.velocity))

    );
    }
    public Command profile(double lowerPos, double upperPos, ArmSubsystem armSubsystem){
        upperPos -= armSubsystem.getUpperPosition();
        lowerPos -= armSubsystem.getLowerPosition();
        Command lower = lowerCommand(lowerPos, armSubsystem);
        Command upper = upperCommand(upperPos, armSubsystem);
        ParallelCommandGroup group = new ParallelCommandGroup(lower, upper);
        group.addRequirements(armSubsystem);
        return group;
    }
    public Command profilePreset(Preset preset, ArmSubsystem armSubsystem){
        double lowerPos = armSubsystem.getMap().get(preset).theta1;
        double upperPos = armSubsystem.getMap().get(preset).theta2;
        upperPos -= armSubsystem.getUpperPosition();
        lowerPos -= armSubsystem.getLowerPosition();
        ArmProfileCommandLower lower = new ArmProfileCommandLower(lowerPos, armSubsystem);
        ArmProfileCommandUpper upper = new ArmProfileCommandUpper(upperPos, armSubsystem);
        ParallelCommandGroup group = new ParallelCommandGroup(lower, upper);
        group.addRequirements(armSubsystem);
        return group;
    }
}
