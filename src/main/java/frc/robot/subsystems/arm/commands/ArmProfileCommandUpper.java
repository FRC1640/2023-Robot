package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmProfileCommandUpper extends TrapezoidProfileCommand {

    
  
    public ArmProfileCommandUpper(double upperPos, ArmSubsystem armSubsystem) {
      
      super(
  
          new TrapezoidProfile(
  
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

    @Override
    public void initialize() {

    }
  
  }