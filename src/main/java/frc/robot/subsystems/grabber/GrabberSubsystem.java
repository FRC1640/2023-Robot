package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase{
    Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    Servo cymbalServo = new Servo(0); 

    public static double CYMBAL_SERVO_HIGH_ANGLE = 0;
    public static double CYMBAL_SERVO_MID_ANGLE = 60;
    public static double CYMBAL_SERVO_UPRIGHT_ANGLE = 90; // 60


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
            cymbalServo.setAngle(CYMBAL_SERVO_MID_ANGLE);
        }
        System.out.println("POST ANGLE: " + cymbalServo.getAngle());
    }
    public void toggleServoTurned(){
       
        if (cymbalServo.getAngle() > 67){ // if servo angle is at 90 (low), set mid
            // cymbalServo.set(.5);
            cymbalServo.setAngle(CYMBAL_SERVO_MID_ANGLE);
        } else if (cymbalServo.getAngle() >= 45){ // if servo angle is 45 < angle < 67 (at mid) set high
            cymbalServo.setAngle(CYMBAL_SERVO_HIGH_ANGLE);
        }
        else{ // servo angle is is less than 45 (at high) set to low
            cymbalServo.setAngle(CYMBAL_SERVO_UPRIGHT_ANGLE);
        }

    }

    public void toggleClamped() {
        solenoid.set(!solenoid.get());
    }
}
