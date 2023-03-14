package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

public class PixyCam {
    double[] three = {0.0, 0.0, 0.0};
    AnalogInput pixyCam = new AnalogInput(6);

    public void getHPint(){
        three[2] = three[1];
        three[1] = three[0];
        three[0] = pixyCam.getVoltage();
        double avg = (three[2] + three[1] + three[0])/3.0;
        if(avg>1.5 && avg<2.3){
            System.out.print("Green");
        } else {
            System.out.print("Red");
        }
        System.out.println(avg);
        //return (int)Math.round(pixyCam.getVoltage());
    }
    public double getHPposition(){
        return pixyCam.getVoltage();
    }
}
