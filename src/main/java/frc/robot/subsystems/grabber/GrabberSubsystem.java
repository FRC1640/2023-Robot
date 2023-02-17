package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase{
    Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    public GrabberSubsystem(){

    }

    public void setClamped(boolean clamped){
        solenoid.set(clamped);
    }
}
