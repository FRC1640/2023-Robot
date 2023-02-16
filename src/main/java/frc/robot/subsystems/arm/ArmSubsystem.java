package frc.robot.subsystems.arm;

import java.util.EnumMap;
import java.util.Map;

import javax.print.attribute.standard.PresentationDirection;
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

public class ArmSubsystem extends SubsystemBase {
    // TODO: set upper arm ids
    // TODO: arm encoders
    // TODO: figure out button bindings
    //TODO: LIMITS!!!!!
    CANSparkMax lowerArmMotor1 = new CANSparkMax(15, MotorType.kBrushless);
    CANSparkMax lowerArmMotor2 = new CANSparkMax(14, MotorType.kBrushless);
    CANSparkMax upperArmMotor1 = new CANSparkMax(6, MotorType.kBrushless);
    CANSparkMax upperArmMotor2 = new CANSparkMax(7, MotorType.kBrushless);

    final double lowerArmMaxSpeed = 0;
    final double lowerArmMaxAccel = 0;
    final double upperArmMaxSpeed = 0;
    final double upperArmMaxAccel = 0;

    double lowerArmSpeed = 0;
    double upperArmSpeed = 0;

    Preset currentPreset = Preset.Ground;

    public static enum Preset {
        Ground,
        C5,
        LowCube,
        HighCube,
        LowCone,
        HighCone;
    }

    static final Map<Preset, ArmState> presetMap = new EnumMap<>(Map.ofEntries( // TODO: Points
            Map.entry(Preset.Ground, ArmState.fromEndEffector(0, 0)),
            Map.entry(Preset.C5, ArmState.fromEndEffector(0, 0)),
            Map.entry(Preset.LowCube, ArmState.fromEndEffector(0, 0)),
            Map.entry(Preset.HighCube, ArmState.fromEndEffector(0, 0)),
            Map.entry(Preset.LowCone, ArmState.fromEndEffector(0, 0)),
            Map.entry(Preset.HighCone, ArmState.fromEndEffector(0, 0))));

    public ArmSubsystem() {
        lowerArmMotor2.follow(lowerArmMotor1);
        upperArmMotor2.follow(upperArmMotor1);
    }

    @Override
    public void periodic() {
        // System.out.format("%.2f, %.2f\n", upperArmSpeed, lowerArmSpeed);
        lowerArmMotor1.set(lowerArmSpeed);
        upperArmMotor1.set(upperArmSpeed);
    }

    public double getLowerArmMaxSpeed() {
        return lowerArmMaxSpeed;
    }

    public double getUpperArmMaxSpeed() {
        return upperArmMaxSpeed;
    }

    public double getLowerArmMaxAccel() {
        return lowerArmMaxAccel;
    }

    public double getUpperArmMaxAccel() {
        return upperArmMaxAccel;
    }

    public void setPreset(Preset preset) {
        currentPreset = preset;
    }

    public Preset getPreset() {
        return currentPreset;
    }
    //TODO: return angle, not encoder count
    public double getLowerPosition() {
        return lowerArmMotor1.getEncoder().getPosition();
    }

    public double getUpperPosition() {
        return upperArmMotor1.getEncoder().getPosition();
    }

    public void stopArm() {
        lowerArmSpeed = 0;
        upperArmSpeed = 0;
    }

    public void setSpeedLower(double speed){
        lowerArmSpeed = speed;
    }
    public void setSpeedUpper(double speed){
        upperArmSpeed = speed;
    }

    public Command armProfile(double lowerPos, double upperPos, double startPositionLower, double startPositionUpper) {
        TrapezoidProfileCommand lowerProfile = new TrapezoidProfileCommand(
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                getLowerArmMaxSpeed(), getLowerArmMaxAccel()),
                        new TrapezoidProfile.State(lowerPos - startPositionLower, 0)),
                state -> setSpeedLower(state.velocity / getLowerArmMaxSpeed()));

        // upper profile
        TrapezoidProfileCommand upperProfile = new TrapezoidProfileCommand(
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                getUpperArmMaxSpeed(), getUpperArmMaxAccel()),
                        new TrapezoidProfile.State(upperPos - startPositionUpper, 0)),
                state -> setSpeedUpper(state.velocity / getUpperArmMaxSpeed()));
        ParallelCommandGroup group = new ParallelCommandGroup(lowerProfile, upperProfile);
        group.addRequirements(this);
        return group;
    }

    public Command armProfilePreset(Preset preset){
        return armProfile(presetMap.get(preset).theta1, presetMap.get(preset).theta2, getLowerPosition(), getUpperPosition());
    }

    public Command tripleMove(double lowerPos, double upperPos) {
        // TODO: set values
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