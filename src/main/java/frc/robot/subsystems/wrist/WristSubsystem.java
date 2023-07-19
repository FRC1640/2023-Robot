package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase{
    PIDController wristController = new PIDController(0, 0, 0);
    CANSparkMax wristMotor = new CANSparkMax(12, MotorType.kBrushless);
    final double wristMax =99; //TODO: set these
    final double wristMin = -99;



    public WristSubsystem(){
        
    }
    public double getWristPosition(){
        return wristMotor.getEncoder().getPosition();
    }
    public void runWrist(double speed){
        if (getWristPosition() >= wristMax){
            speed = Math.max(0, speed);
        }
        if (getWristPosition() <= wristMin){
            speed = Math.min(0, speed);
        }
        wristMotor.set(speed);
    }

    public void runWristToPosition(double position){
        //TODO: this
        wristMotor.set(wristController.calculate(getWristPosition(), position));

    }
    



}
