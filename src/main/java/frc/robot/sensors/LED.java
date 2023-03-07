package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

public class LED {
    DigitalOutput blue = new DigitalOutput(8);
    DigitalOutput green = new DigitalOutput(9);
    public void setStateBlue(){
        blue.set(true);
        green.set(false);
    }
    public void setStateGreen(){
        blue.set(false);
        green.set(true);
    }
}
