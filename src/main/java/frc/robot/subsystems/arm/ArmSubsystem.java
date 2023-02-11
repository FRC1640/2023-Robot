package frc.robot.subsystems.arm;

import javax.swing.TransferHandler.TransferSupport;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

public class ArmSubsystem extends SubsystemBase{
    //TODO: set ids swap ids
    CANSparkMax lowerArmMotor1 = new CANSparkMax(15, MotorType.kBrushless);
    CANSparkMax lowerArmMotor2 = new CANSparkMax(14, MotorType.kBrushless);
    // CANSparkMax upperArmMotor = new CANSparkMax(0, MotorType.kBrushless);
    
    double lowerArmSpeedManual = 0;
    double upperArmSpeedManual = 0;

    double lowerArmSpeedPreset = 0;
    double upperArmSpeedPreset = 0;

    final double lowerArmMaxSpeed = 0;
    final double lowerArmMaxAccel = 0;
    final double upperArmMaxSpeed = 0;
    final double upperArmMaxAccel = 0;
    

    Preset currentPreset = Preset.Down;
    public static enum ArmMode {
        Manual,
        EndEffector,
        Preset;
        
    }
    
    public static enum Preset{
        Down(0, 0),
        Up(1, 2, null);

        public double lowerArmAngle;
        public double upperArmAngle;
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
        lowerArmMotor2.follow(lowerArmMotor1);
        
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
            lowerArmSpeed = lowerArmSpeedPreset;

        }
        System.out.format("%.2f, %.2f\n", upperArmSpeed, lowerArmSpeed);
        lowerArmMotor1.set(lowerArmSpeed);
        // upperArmMotor.set(upperArmSpeed);
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
    public void setPresetSpeedLower(double speed){
        lowerArmSpeedPreset = speed;
    }
    public void setPresetSpeedUpper(double speed){
        upperArmSpeedPreset = speed;
    }
    public double getLowerArmMaxSpeed(){
        return lowerArmMaxSpeed;
    }
    public double getUpperArmMaxSpeed(){
        return upperArmMaxSpeed;
    }
    public double getLowerArmMaxAccel(){
        return lowerArmMaxAccel;
    }
    public double getUpperArmMaxAccel(){
        return upperArmMaxAccel;
    }
    public void setPreset(Preset preset){
        currentPreset = preset;
    }
    public Preset getPreset(){
        return currentPreset;
    }
    public double getLowerPosition(){
        return lowerArmMotor1.getEncoder().getPosition();
    }
    public double getUpperPosition(){
        // return upperArmMotor.getEncoder().getPosition();
        return 0;
    }


    public Command armProfile(double lowerPos, double upperPos){
        TrapezoidProfileCommand lowerProfile = new TrapezoidProfileCommand(
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    getLowerArmMaxSpeed(), getLowerArmMaxAccel()),
                     new TrapezoidProfile.State(lowerPos - getLowerPosition(), 0)),
                     state -> setPresetSpeedLower(state.velocity / getLowerArmMaxSpeed()));
        
        //upper profile
        TrapezoidProfileCommand upperProfile = new TrapezoidProfileCommand(
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    getUpperArmMaxSpeed(), getUpperArmMaxAccel()),
                     new TrapezoidProfile.State(upperPos - getUpperPosition(), 0)),
                      state -> setPresetSpeedUpper(state.velocity / getUpperArmMaxSpeed()));
        ParallelCommandGroup group = new ParallelCommandGroup(lowerProfile, upperProfile);
        group.addRequirements(this);
        return group;
    }
}