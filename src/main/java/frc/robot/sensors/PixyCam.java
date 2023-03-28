package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.sensors.Limelight.LedEnum;

public class PixyCam {
    double[] three = {0.0, 0.0, 0.0};
    AnalogInput pixyCam = new AnalogInput(6);
    LED led;
    public PixyCam(LED led){
        this.led = led;
    }
    public void getHPint(){
        three[2] = three[1];
        three[1] = three[0];
        three[0] = pixyCam.getVoltage();
        double avg = (three[2] + three[1] + three[0])/3.0;
        if(avg>1.5 && avg<2.3){
            led.setHPGreen();
        } else {
            led.setHPRed();
        }
        // System.out.println(avg);
        //return (int)Math.round(pixyCam.getVoltage());
    }
    public double getHPposition(){
        return pixyCam.getVoltage();
    }
}
