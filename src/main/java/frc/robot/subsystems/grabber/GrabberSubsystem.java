package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase{
    Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    Servo cymbalServo = new Servo(0); //TODO: change channel

    public static double CYMBAL_SERVO_TURNED_ANGLE = 180;


    public GrabberSubsystem(){

    }

    public void setClamped(boolean clamped){
        solenoid.set(clamped);
    }

    public void setServoTurned(boolean turned){
        if (turned == false){
            // cymbalServo.set(.5);
            cymbalServo.setAngle(0);
        } else{
            cymbalServo.setAngle(CYMBAL_SERVO_TURNED_ANGLE);
        }
    }

    public void toggleClamped() {
        solenoid.set(!solenoid.get());
    }
}
