package frc.robot.subsystems.grabber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase{
    DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);  
    CANSparkMax wheelLeft = new CANSparkMax(11, MotorType.kBrushless);
    CANSparkMax wheelRight = new CANSparkMax(12, MotorType.kBrushless);
    public GrabberSubsystem(){
        setClamped(true);
    }

    public void setRollerSpeed(double motorSpeed){
        wheelLeft.set(motorSpeed);
        wheelRight.set(-motorSpeed);
    }

    public void setClamped(boolean clamped){
        if (clamped) {
            solenoid.set(Value.kReverse);
        }
        else {
            solenoid.set(Value.kForward);
        }
    }

    public boolean getClamped(){
        return solenoid.get() == Value.kReverse;
    }

    public void toggleClamped() {
        setClamped(!getClamped());
    }
}
