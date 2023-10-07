package frc.robot.subsystems.grabber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem.Preset;

public class GrabberSubsystem extends SubsystemBase{
    DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0); 
    CANSparkMax rollerMotor = new CANSparkMax(11, MotorType.kBrushless);
    Servo cymbalServo = new Servo(0); 
    double servoOffset;
    RobotContainer robotContainer;
    double[] rollerCurrentAverage = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0};



    public GrabberSubsystem(RobotContainer robotContainer){
        setClamped(true); 
        this.robotContainer = robotContainer;
        rollerMotor.setOpenLoopRampRate(0.5);
        rollerMotor.setClosedLoopRampRate(0.5);
    }

    public void setClamped(boolean clamped){
        if (clamped){
            solenoid.set(Value.kReverse);
        }
        else{
            cymbalServo.setAngle(Constants.ServoSmasAngles.CYMBAL_SERVO_UPRIGHT_ANGLE); // reset servo after release
            servoOffset = 0;
            robotContainer.setGround(0);
            solenoid.set(Value.kForward);
        }
    }
    public boolean getClamped(){
        if (solenoid.get() == Value.kReverse){
            return true;
        }
        else{
            return false;
        }
    }

    public void setServoTurned(boolean turned){
        // System.out.println("PRE ANGLE: "+ cymbalServo.getAngle()+ " INPUT: "+ turned);
        if (turned == false){
            // cymbalServo.set(.5);
            cymbalServo.setAngle(Constants.ServoSmasAngles.CYMBAL_SERVO_UPRIGHT_ANGLE);
        } else{
            cymbalServo.setAngle(Constants.ServoSmasAngles.CYMBAL_SERVO_MID_ANGLE);
        }
        // System.out.println("POST ANGLE: " + cymbalServo.getAngle());
    }
   
    public void toggleServoTurned(){
       
        if (cymbalServo.getAngle() > 67){ // if servo angle is at 90 (low), set mid
            // cymbalServo.set(.5);
            cymbalServo.setAngle(Constants.ServoSmasAngles.CYMBAL_SERVO_MID_ANGLE);
        } else if (cymbalServo.getAngle() >= 45){ // if servo angle is 45 < angle < 67 (at mid) set high
            cymbalServo.setAngle(Constants.ServoSmasAngles.HIGH_ANGLE);
        }
        else{ // servo angle is is less tha n 45 (at high) set to low
            cymbalServo.setAngle(Constants.ServoSmasAngles.CYMBAL_SERVO_UPRIGHT_ANGLE);
        }

    }

    public void updateRollerCurrentAverage(){
        double[] averageList = rollerCurrentAverage;
        for (int i = 0; i < rollerCurrentAverage.length - 1; i++){
            rollerCurrentAverage[i + 1] = averageList[i];
        }
        rollerCurrentAverage[0] = getRollerCurrent();

    }
    public double getRollerCurrentAverage(){
        double average = 0;
        for (int i = 0; i < rollerCurrentAverage.length; i++){
            average += rollerCurrentAverage[i];
        }
        average = average / rollerCurrentAverage.length;
        return average;
    }

    public void servoMove(double angle){
        cymbalServo.setAngle(angle);
    }

    public void setServoOffset(double newOffset){
        servoOffset = newOffset;
    }

    public double getServoOffset(){
        return servoOffset;
    }

    
    public void setServoAngle(double newAngle){ // mar 17 changes
        cymbalServo.setAngle(newAngle);
    }


    // public void toggleClamped() {
    //     boolean wasClamped = getClamped();
    //     setClamped(!wasClamped);
    //     if (wasClamped) {
    //         cymbalServo.setAngle(Constants.ServoSmasAngles.CYMBAL_SERVO_UPRIGHT_ANGLE); // reset servo after release
    //         servoOffset = 0;
    //         robotContainer.setGround(0);
    //     }
    //     else{
    //         if (robotContainer.getCurrentPreset() == Preset.Ground){
    //             robotContainer.setGround(1);
    //         }
    //         // if (robotContainer.getCurrentPreset() == Preset.UprightConeGround){
    //         //     robotContainer.setGround(2);
    //         // }
    //     }
    // }

    public void incramentServoUp(){
        if (cymbalServo.getAngle() < 176){
            cymbalServo.setAngle(cymbalServo.getAngle() + 5);
        }
    }  

    public void incramentServoDown(){
        if (cymbalServo.getAngle() > 4){
            cymbalServo.setAngle(cymbalServo.getAngle() - 5);
        }        
        
    }
    public void spinGrabber(double speed){
        rollerMotor.set(speed);
    }

    public double getRollerCurrent(){
        return (rollerMotor.getOutputCurrent());
    }

    @Override
    public void periodic() {
        // System.out.println(cymbalServo.getAngle());
        updateRollerCurrentAverage();
        // System.out.println("Average: " + getRollerCurrentAverage());
    }
}
