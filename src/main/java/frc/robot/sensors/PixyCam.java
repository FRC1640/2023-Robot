package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

public class PixyCam {
    AnalogInput pixyCam = new AnalogInput(6);//TODO: set ID
    public void getHPint(){
        if(pixyCam.getVoltage()>1.2 && pixyCam.getVoltage()<1.8){
            System.out.println("Green");
        } else {
            System.out.println("Red");
        }
        //return (int)Math.round(pixyCam.getVoltage());
    }
    public double getHPposition(){
        return pixyCam.getVoltage();
    }
}
