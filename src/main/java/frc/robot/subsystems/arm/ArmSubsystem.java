package frc.robot.subsystems.arm;

import java.util.EnumMap;
import java.util.Map;

import javax.print.attribute.standard.PresentationDirection;
import javax.swing.TransferHandler.TransferSupport;

import org.ejml.dense.row.mult.VectorVectorMult_CDRM;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.Resolver;

public class ArmSubsystem extends SubsystemBase {
    // TODO: arm encoders
    // TODO: figure out button bindings
    //TODO: LIMITS!!!!!
    //TODO: max speeds
    //TOODO: feedforward
    CANSparkMax lowerArmMotor1 = new CANSparkMax(15, MotorType.kBrushless);
    CANSparkMax lowerArmMotor2 = new CANSparkMax(14, MotorType.kBrushless);
    CANSparkMax upperArmMotor1 = new CANSparkMax(6, MotorType.kBrushless);
    CANSparkMax upperArmMotor2 = new CANSparkMax(7, MotorType.kBrushless);

    final double lowerArmMaxSpeed = 0;
    final double lowerArmMaxAccel = 0;
    final double upperArmMaxSpeed = 0;
    final double upperArmMaxAccel = 0;

    double lowerArmVoltage = 0;
    double upperArmVoltage = 0;


    SimpleMotorFeedforward lowerFF = new SimpleMotorFeedforward(0, 0, 0);
    SimpleMotorFeedforward upperFF = new SimpleMotorFeedforward(0, 0, 0);
    //TODO: ports
    Resolver lowerEncoder;
    Resolver upperEncoder;



    final double lowerP = 0;
    final double lowerI = 0;
    final double lowerD = 0;
    final double upperP = 0;
    final double upperI = 0;
    final double upperD = 0;

    final double lowerArmMin = -17; //-17
    final double lowerArmMax = 45; //45
    final double upperArmMin = 10; //10
    final double upperArmMax = 175; //165
    

    Preset currentPreset = Preset.Ground;
    public static enum Preset {
        Ground,
        C5,
        LowCube,
        HighCube,
        LowCone,
        HighCone;
    }

    static final Map<Preset, ArmState> presetMap = null;
    
    // new EnumMap<>(Map.ofEntries( // TODO: Points
    //         Map.entry(Preset.Ground, ArmState.fromEndEffector(0, 0)),
    //         Map.entry(Preset.C5, ArmState.fromEndEffector(0, 0)),
    //         Map.entry(Preset.LowCube, ArmState.fromEndEffector(0, 0)),
    //         Map.entry(Preset.HighCube, ArmState.fromEndEffector(0, 0)),
    //         Map.entry(Preset.LowCone, ArmState.fromEndEffector(0, 0)),
    //         Map.entry(Preset.HighCone, ArmState.fromEndEffector(0, 0))));

    public ArmSubsystem(Resolver lowerEncoder,Resolver upperEncoder) {
        lowerArmMotor2.follow(lowerArmMotor1);
        upperArmMotor2.follow(upperArmMotor1);
        this.lowerEncoder = lowerEncoder;
        this.upperEncoder = upperEncoder;
        lowerArmMotor1.setIdleMode(IdleMode.kBrake);
        lowerArmMotor2.setIdleMode(IdleMode.kBrake);
        upperArmMotor1.setIdleMode(IdleMode.kBrake);
        upperArmMotor2.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        if (getLowerPosition() >= lowerArmMax){
            lowerArmVoltage = Math.max(lowerArmVoltage, 0);
        }
        if (getLowerPosition() <= lowerArmMin){
            lowerArmVoltage = Math.min(lowerArmVoltage, 0);
        }
        
        if (getUpperPosition() >= upperArmMax){
            upperArmVoltage = Math.max(upperArmVoltage, 0);
        }
        if (getUpperPosition() <= upperArmMin){
            upperArmVoltage = Math.min(upperArmVoltage, 0);
        }
        
        lowerArmMotor1.setVoltage(lowerArmVoltage);
        upperArmMotor1.setVoltage(upperArmVoltage);
        
        // System.out.println(lowerArmSpeed + ", " + upperArmSpeed);
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
        double a =lowerEncoder.getD();
        if (a >= 180){
            a-=360;
        }
        return a;
    }

    public double getUpperPosition() {
        double a =upperEncoder.getD();
        if (a >= 180){
            a-=360;
        }
        return a;
    }

    public void stopArm() {
        lowerArmVoltage = 0;
        upperArmVoltage = 0;
    }

    public void setSpeedLower(double speed){
        lowerArmVoltage = speed * 12;
    }
    public void setSpeedUpper(double speed){
        upperArmVoltage = speed * 12;
    }

    public void setLowerVoltage(double voltage){
        lowerArmVoltage =voltage;
    }
    public void setUpperVoltage(double voltage){
        upperArmVoltage = voltage;
    }

    public double getLowerFFVoltageAccel(double velocity, double accel){
        return lowerFF.calculate(velocity, accel);
    }

    public double getUpperFFVoltageAccel(double velocity, double accel){
        return upperFF.calculate(velocity, accel);
    }
    
    public double getLowerFFVoltage(double velocity){
        return lowerFF.calculate(velocity);
    }

    public double getUpperFFVoltage(double velocity){
        return upperFF.calculate(velocity);
    }
    public ProfiledPIDController createControllerLower(){
        return new ProfiledPIDController(lowerP,lowerI,lowerD, new TrapezoidProfile.Constraints(lowerArmMaxSpeed, lowerArmMaxAccel));
    }
    public ProfiledPIDController createControllerUpper(){
        return new ProfiledPIDController(upperP,upperI,upperD, new TrapezoidProfile.Constraints(upperArmMaxSpeed, upperArmMaxAccel));
    }
    public Command armProfile(double posLower, double posUpper) {
        ProfiledPIDCommand commandLower = new ProfiledPIDCommand(createControllerLower(), () -> getLowerPosition(), new TrapezoidProfile.State(posLower, 0), (pidV, trapState) -> setLowerVoltage(pidV + trapState.velocity));
        ProfiledPIDCommand commandUpper = new ProfiledPIDCommand(createControllerUpper(), () -> getUpperPosition(), new TrapezoidProfile.State(posUpper, 0), (pidV, trapState) -> setLowerVoltage(pidV + trapState.velocity));
        ParallelCommandGroup group = new ParallelCommandGroup(commandLower, commandUpper);
        group.addRequirements(this);
        return group;
    }

    public Command armProfilePreset(Preset preset){
        return armProfile(presetMap.get(preset).theta1, presetMap.get(preset).theta2);
    }

    public Translation2d getEndEffectorPosition(){
        ArmKinematics kinematics = new ArmKinematics(Units.inchesToMeters(39), Units.inchesToMeters(35.4));
        kinematics.setLowerAngle(Math.toRadians(getLowerPosition()));
        kinematics.setUpperAngle(Math.toRadians(getUpperPosition()));
        kinematics.fowardKinematics();
        Translation2d point = new Translation2d(kinematics.getX(), kinematics.getY());
        return point;
    }

    // public Command tripleMove(double lowerPos, double upperPos) {
    //     // TODO: set values
    //     double lowerPos1 = -1;
    //     double upperPos1 = getUpperPosition();
    //     double lowerPos2 = getLowerPosition();
    //     double upperPos2 = -1;
    //     double lowerPos3 = lowerPos;
    //     double upperPos3 = upperPos;
    //     Command profile1 = armProfile(lowerPos1, upperPos1, getLowerPosition(), getUpperPosition());
    //     Command profile2 = armProfile(lowerPos2, upperPos2, lowerPos1, upperPos1);
    //     Command profile3 = armProfile(lowerPos3, upperPos3, lowerPos2, upperPos2);
    //     SequentialCommandGroup group = new SequentialCommandGroup(profile1, profile2, profile3);
    //     return group;
    // }
}