package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase{
    Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    Servo cymbalServo = new Servo(0); 

    public static double CYMBAL_SERVO_TURNED_ANGLE = 0;
    public static double CYMBAL_SERVO_UPRIGHT_ANGLE = 60;


    public GrabberSubsystem(){

    }

    public void setClamped(boolean clamped){
        solenoid.set(clamped);
    }

    public void setServoTurned(boolean turned){
        System.out.println("PRE ANGLE: "+ cymbalServo.getAngle()+ " INPUT: "+ turned);
        if (turned == false){
            // cymbalServo.set(.5);
            cymbalServo.setAngle(CYMBAL_SERVO_UPRIGHT_ANGLE);
        } else{
            cymbalServo.setAngle(CYMBAL_SERVO_TURNED_ANGLE);
        }
        System.out.println("POST ANGLE: " + cymbalServo.getAngle());
    }
    public void toggleServoTurned(){
       
        if (cymbalServo.getAngle() > 20){ // if servo angle is at 60, set high
            // cymbalServo.set(.5);
            cymbalServo.setAngle(CYMBAL_SERVO_TURNED_ANGLE);
        } else{
            cymbalServo.setAngle(CYMBAL_SERVO_UPRIGHT_ANGLE);
        }

    }

    public void toggleClamped() {
        solenoid.set(!solenoid.get());
    }
}
