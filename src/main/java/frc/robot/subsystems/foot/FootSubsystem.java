package frc.robot.subsystems.foot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FootSubsystem extends SubsystemBase{
   DoubleSolenoid footSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

    public FootSubsystem(){
        footSolenoid.set(Value.kForward);
    }

    public void toggleClamped() {
        
        footSolenoid.toggle();
    }
}
