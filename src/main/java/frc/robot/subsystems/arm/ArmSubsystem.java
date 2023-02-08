package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    public enum ArmMode {
        manual(0),
        endEffector(1),
        preset(2);

        public final int value;
    
        ArmMode(int value) {
          this.value = value;
        }
    }
    ArmMode mode;
    @Override
    public void periodic() {
        
        super.periodic();
    }
    public void setMode(ArmMode newMode){
        mode = newMode;
    }
}