package frc.robot.subsystems.arm;

import java.util.EnumMap;
import java.util.Map;

import javax.swing.TransferHandler.TransferSupport;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

public class ArmSubsystem extends SubsystemBase{
    //TODO: set ids
    CANSparkMax lowerArmMotor1 = new CANSparkMax(15, MotorType.kBrushless);
    CANSparkMax lowerArmMotor2 = new CANSparkMax(14, MotorType.kBrushless);
    CANSparkMax upperArmMotor1 = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax upperArmMotor2 = new CANSparkMax(0, MotorType.kBrushless);
    
    double lowerArmSpeedManual = 0;
    double upperArmSpeedManual = 0;

    double lowerArmSpeedPreset = 0;
    double upperArmSpeedPreset = 0;

    double lowerArmSpeedEndEffector = 0;
    double upperArmSpeedEndEffector = 0;
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
        Down(),
        Up();
    }
    // Map<Preset, ArmState> presetMap = new EnumMap<Preset, ArmState>();
    //TODO: Map
    ArmMode mode;
    public ArmSubsystem(){
        lowerArmMotor2.follow(lowerArmMotor1);
        upperArmMotor2.follow(upperArmMotor1);
    }
    @Override
    public void periodic() {
        double lowerArmSpeed = 0;
        double upperArmSpeed = 0;
        if (mode == ArmMode.Manual){
            lowerArmSpeed = lowerArmSpeedManual;
            upperArmSpeed = upperArmSpeedManual;
            
        }

        if (mode == ArmMode.Preset){
            lowerArmSpeed = lowerArmSpeedPreset;

        }

        if (mode == ArmMode.EndEffector){
            lowerArmSpeed = lowerArmSpeedEndEffector;
            upperArmSpeed = upperArmSpeedEndEffector;
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
    public void setEndEffectoLower(double speed){
        lowerArmSpeedEndEffector = speed;
    }
    public void setEndEffectorUpper(double speed){
        upperArmSpeedEndEffector = speed;
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


    public Command armProfile(double lowerPos, double upperPos, double startPositionLower, double startPositionUpper){
        TrapezoidProfileCommand lowerProfile = new TrapezoidProfileCommand(
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    getLowerArmMaxSpeed(), getLowerArmMaxAccel()),
                     new TrapezoidProfile.State(lowerPos - startPositionLower, 0)),
                     state -> setPresetSpeedLower(state.velocity / getLowerArmMaxSpeed()));
        
        //upper profile
        TrapezoidProfileCommand upperProfile = new TrapezoidProfileCommand(
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    getUpperArmMaxSpeed(), getUpperArmMaxAccel()),
                     new TrapezoidProfile.State(upperPos - startPositionUpper, 0)),
                      state -> setPresetSpeedUpper(state.velocity / getUpperArmMaxSpeed()));
        ParallelCommandGroup group = new ParallelCommandGroup(lowerProfile, upperProfile);
        group.addRequirements(this);
        return group;
    }


    public Command tripleMove(double lowerPos, double upperPos){
        //TODO: set values
        double lowerPos1 = -1;
        double upperPos1 = getUpperPosition();
        double lowerPos2 = getLowerPosition();
        double upperPos2 = -1;
        double lowerPos3 = lowerPos;
        double upperPos3 = upperPos;
        Command profile1 = armProfile(lowerPos1, upperPos1, getLowerPosition(), getUpperPosition());
        Command profile2 = armProfile(lowerPos2, upperPos2, lowerPos1, upperPos1);
        Command profile3 = armProfile(lowerPos3, upperPos3, lowerPos2, upperPos2);
        SequentialCommandGroup group = new SequentialCommandGroup(profile1, profile2, profile3);
        return group;
    }
}