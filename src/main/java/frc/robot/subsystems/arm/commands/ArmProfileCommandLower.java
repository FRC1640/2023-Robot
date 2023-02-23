package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmProfileCommandLower extends TrapezoidProfileCommand {
  
    public ArmProfileCommandLower(double lowerPos, ArmSubsystem armSubsystem) {
  
      super(
  
          new TrapezoidProfile(
  
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
  
  }