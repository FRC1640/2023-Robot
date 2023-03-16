package frc.robot.sensors;
import edu.wpi.first.wpilibj.DigitalOutput;

public class LED {
    DigitalOutput blue = new DigitalOutput(8);
    DigitalOutput green = new DigitalOutput(9);
    DigitalOutput HP1 = new DigitalOutput(0); //TODO: set id
    DigitalOutput HP2 = new DigitalOutput(0); //TODO: set id
    public void setStateBlue(){
        blue.set(true);
        green.set(false);
    }
    public void setStateGreen(){
        blue.set(false);
        green.set(true);
    }
    public void setHPGreen(){
        HP1.set(true);
        HP2.set(false);
    }
    public void setHPRed(){
        HP1.set(false);
        HP2.set(true);
    }
}
