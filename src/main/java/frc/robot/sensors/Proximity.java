package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class Proximity {
    DigitalInput proximitySensor = new DigitalInput(0);
    Boolean c5status = false;
    Boolean lastRead = false;
    Boolean seesNew = false;

    public boolean getC5boolean(){
        c5status = proximitySensor.get();
        // System.out.println(c5status);
        if(!lastRead && c5status){
            seesNew = true;
        } else {
            seesNew = false;
        }
        lastRead = c5status;
        return seesNew;
    }
    
}
