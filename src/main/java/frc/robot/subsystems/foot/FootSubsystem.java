package frc.robot.subsystems.foot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;


public class FootSubsystem extends SubsystemBase{
   DoubleSolenoid footSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
   DigitalInput footLimitSwitch = new DigitalInput(9999); // uh unknown channel

    public FootSubsystem(){
        footSolenoid.set(Value.kForward);
    }
    public boolean getFootDown(){
        return footSolenoid.get() == Value.kReverse;
    }

    public boolean getLimitSwitch(){
        return footLimitSwitch.get();
    }

    public void toggleClamped() {
        footSolenoid.toggle();
    }
}
