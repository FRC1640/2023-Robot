package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    //TODO: set ids
    CANSparkMax lowerArmMotor = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax upperArmMotor = new CANSparkMax(0, MotorType.kBrushless);
    
    double lowerArmSpeedManual = 0;
    double upperArmSpeedManual = 0;
    public static enum ArmMode {
        Manual,
        EndEffector,
        Preset;
    }

    public static enum Preset{
        Down(0, 0),
        Up(1, 2, null);

        double lowerArmAngle;
        double upperArmAngle;

        Preset(double lowerArmAngle, double upperArmAngle) {
            this.lowerArmAngle = lowerArmAngle;
            this.upperArmAngle = upperArmAngle;
        }

        Preset(double x, double y, Object donotuse) {
            //TODO: MATH
        }
    }
    ArmMode mode;
    public ArmSubsystem(){

    }
    @Override
    public void periodic() {
        double lowerArmSpeed = 0;
        double upperArmSpeed = 0;
        if (mode == ArmMode.Manual){
            lowerArmSpeed = lowerArmSpeedManual;
            upperArmSpeed = upperArmSpeedManual;
        }
        else{
            lowerArmSpeed = 0;
            upperArmSpeed = 0;
        }

        if (mode == ArmMode.Preset){

        }

        lowerArmMotor.set(lowerArmSpeed);
        upperArmMotor.set(upperArmSpeed);
    }
    public void setMode(ArmMode newMode){
        mode = newMode;
        lowerArmSpeedManual = 0;
        upperArmSpeedManual = 0;
    }
    public void setManualUpper(double speed){
        upperArmSpeedManual = speed;
    }
    public void setManualLower(double speed){
        lowerArmSpeedManual = speed;
    }
    
}