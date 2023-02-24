package frc.robot.subsystems.arm;

import java.util.EnumMap;
import java.util.Map;

import javax.print.attribute.standard.PresentationDirection;
import javax.swing.TransferHandler.TransferSupport;
import javax.swing.text.html.StyleSheet;

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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.sensors.Resolver;
import frc.robot.subsystems.arm.commands.ArmStopCommand;

public class ArmSubsystem extends SubsystemBase {
    //TODO: LIMITS!!!!!
    //TODO: max speeds
    //TOODO: feedforward
    /*
     * GEAR RATIO:
     * Shoulder: 580
     * Elbow: 498
     */
    CANSparkMax lowerArmMotor1 = new CANSparkMax(15, MotorType.kBrushless);
    CANSparkMax lowerArmMotor2 = new CANSparkMax(14, MotorType.kBrushless);
    CANSparkMax upperArmMotor1 = new CANSparkMax(6, MotorType.kBrushless);
    CANSparkMax upperArmMotor2 = new CANSparkMax(7, MotorType.kBrushless);

    final double lowerArmMaxSpeed = 30;//30
    final double lowerArmMaxAccel = 360;//5
    final double upperArmMaxSpeed = 55;//55
    final double upperArmMaxAccel = 360;//360

    double lowerArmVoltage = 0;
    double upperArmVoltage = 0;


    SimpleMotorFeedforward lowerFF = new SimpleMotorFeedforward(0.19825, 0.19542, 0.0080273);
    SimpleMotorFeedforward upperFF = new SimpleMotorFeedforward(0.12932, 0.16959, 0.0033965);
    //TODO: ports
    Resolver lowerEncoder;
    Resolver upperEncoder;



    final double lowerP = 0.2;//0.016826
    final double lowerI = 0;
    final double lowerD = 0;

    final double upperP = 0.2; //.0014772
    final double upperI = 0;
    final double upperD = 0;

    final double lowerArmMin = -28; //-17
    final double lowerArmMax = 48; //45
    final double upperArmMin = 1; //10
    final double upperArmMax = 180; //165

    

    Preset currentPreset;

    public static enum Preset {
        Ground,
        Pickup,
        Place;
    }

    static final Map<Preset, ArmState> cubeMap =
    new EnumMap<>(Map.ofEntries(
            Map.entry(Preset.Ground, new ArmState(Math.toRadians(15), Math.toRadians(45))),
            Map.entry(Preset.Pickup, ArmState.fromEndEffector(0.2, 0.1))
            ));

    static final Map<Preset, ArmState> coneMap =
    new EnumMap<>(Map.ofEntries(
            Map.entry(Preset.Ground, new ArmState(Math.toRadians(15), Math.toRadians(45))),
            Map.entry(Preset.Pickup, ArmState.fromEndEffector(0.28, 0.18)),
            Map.entry(Preset.Place, ArmState.fromEndEffector(1.43, 1.23))
            ));
    public ArmSubsystem(Resolver lowerEncoder,Resolver upperEncoder) {
        // System.out.println(presetMap.get(Preset.Ground));
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
        // System.out.println(getUpperPosition());
        
        if (getUpperPosition() >= upperArmMax){
            upperArmVoltage = Math.max(upperArmVoltage, 0);
        }
        if (getUpperPosition() <= upperArmMin){
            upperArmVoltage = Math.min(upperArmVoltage, 0);
        }
        


        // if (lowerArmMotor1.getOutputCurrent() > 35 || upperArmMotor1.getOutputCurrent() > 35){
        //     lowerArmVoltage = 0;
        //     upperArmVoltage = 0;
        //     new ArmStopCommand(this).schedule();
        // } 
        // System.out.println("lower arm: " + getLowerPosition() + " target: " + Math.toDegrees(coneMap.get(Preset.Ground).theta1) + " upper arm: " + getUpperPosition() + " target2: " + Math.toDegrees(coneMap.get(Preset.Ground).theta2));
        

        lowerArmMotor1.setVoltage(lowerArmVoltage);
        upperArmMotor1.setVoltage(upperArmVoltage);
        // System.out.println("theta: " + presetMap.get(Preset.Ground).theta2);
        // System.out.println("lower: " + getLowerPosition());
        // System.out.println("Pos: " + getEndEffectorPosition());
        // System.out.println(lowerArmSpeed + ", " + upperArmSpeed);
        // System.out.println("End effector: " + getEndEffectorPosition());
    }

    public Map<Preset, ArmState> getMap(){
        
        return coneMap;
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
        setLowerVoltage(speed * 12);
    }
    public void setSpeedUpper(double speed){
        setUpperVoltage(speed * 12);
    }

    public void setLowerVoltage(double voltage){
        lowerArmVoltage = voltage;
    }
    public void setUpperVoltage(double voltage){
        upperArmVoltage = voltage;
        // ArmKinematics kinematics = new ArmKinematics(Units.inchesToMeters(39), Units.inchesToMeters(35.4));
        // kinematics.setX(getEndEffectorPosition().getX());
        // kinematics.setY(getEndEffectorPosition().getY());
        // kinematics.inverseKinematics();
        
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
        ProfiledPIDController controller = new ProfiledPIDController(lowerP,lowerI,lowerD, new TrapezoidProfile.Constraints(lowerArmMaxSpeed, lowerArmMaxAccel));
        controller.setTolerance(3);
        return controller;
    }
    public ProfiledPIDController createControllerUpper(){
        ProfiledPIDController controller = new ProfiledPIDController(upperP,upperI,upperD, new TrapezoidProfile.Constraints(upperArmMaxSpeed, upperArmMaxAccel));
        controller.setTolerance(3);
        return controller;
    }
    public Command armProfile(double posLower, double posUpper) {

        ProfiledPIDController lowerController = createControllerLower();
        Command commandLower = new ProfiledPIDCommand(lowerController, () -> getLowerPosition(), 
            new TrapezoidProfile.State(posLower, 0), 
                (pidV, trapState) -> setLowerVoltage(-(getLowerFFVoltage(pidV + trapState.velocity))))
            .until(() -> lowerController.atGoal());
        
        
        ProfiledPIDController upperController = createControllerUpper();
        Command commandUpper = new ProfiledPIDCommand(upperController, () -> getUpperPosition(), 
            new TrapezoidProfile.State(posUpper, 0), 
                (pidV, trapState) -> setUpperVoltage(-(getUpperFFVoltage(pidV + trapState.velocity))))
            .until(() -> upperController.atGoal());
        
        
        ParallelCommandGroup group = new ParallelCommandGroup(commandLower, commandUpper);
        group.addRequirements(this);
        return group;
    }

    // public Command armProfile(double posLower, double posUpper) {
    //     TrapezoidProfileCommand commandLower = new TrapezoidProfileCommand(
    //         new TrapezoidProfile(
    //             new TrapezoidProfile.Constraints(lowerArmMaxSpeed, lowerArmMaxAccel),
    //             new TrapezoidProfile.State(posLower, 0),
    //             new TrapezoidProfile.State(getLowerPosition(), 0)
    //         ),
    //         trapState -> setLowerVoltage(-getLowerFFVoltage(trapState.velocity))
    //     );

    //     TrapezoidProfileCommand commandUpper = new TrapezoidProfileCommand(
    //         new TrapezoidProfile(
    //             new TrapezoidProfile.Constraints(upperArmMaxSpeed, upperArmMaxAccel),
    //             new TrapezoidProfile.State(posUpper, 0),
    //             new TrapezoidProfile.State(getUpperPosition(), 0)
    //         ),
    //         trapState -> setUpperVoltage(-getUpperFFVoltage(trapState.velocity))
    //     );
        
    //     ParallelCommandGroup group = new ParallelCommandGroup(commandLower, commandUpper);
    //     group.addRequirements(this);
    //     return group;
    // }

    public Command armProfilePreset(Preset preset, boolean cubeMode){
        System.out.println(cubeMode);
        double degreeThreshold = 5;
        double velocityThreshold = 0.01;
        
        Map<Preset, ArmState> presetMap;
        if (cubeMode == true){
            presetMap = cubeMap;
            System.out.println("CUBE");
        }
        else{
            presetMap = coneMap;
            System.out.println("CONE");

        }
        // System.out.println("theta2: " + Math.toDegrees(presetMap.get(preset).theta2) + " theta2Current: " + getUpperPosition());
        Command profile = armProfile(Math.toDegrees(presetMap.get(preset).theta1), Math.toDegrees(presetMap.get(preset).theta2));
        double endTheta1 = Math.toDegrees(presetMap.get(preset).theta1);
        double endTheta2 = Math.toDegrees(presetMap.get(preset).theta2);
        currentPreset = preset;
        return profile;
        // .until(() -> (Math.abs(upperEncoder.getV()) <= velocityThreshold && 
        //             Math.abs(lowerEncoder.getV()) <= velocityThreshold)).andThen(new PrintCommand("END!!!!!!!!"));
        
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