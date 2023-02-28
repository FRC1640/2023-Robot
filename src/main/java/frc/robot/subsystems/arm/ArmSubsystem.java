package frc.robot.subsystems.arm;

import java.util.EnumMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.Resolver;
import frc.robot.subsystems.arm.commands.ArmStopCommand;

public class ArmSubsystem extends SubsystemBase {
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
    // ArmFeedforward upperFF = new ArmFeedforward(0.14961,0.075105,0.16904,0.0033585);

    Resolver lowerEncoder;
    Resolver upperEncoder;

    final double lowerP = 0.2;//0.016826
    final double lowerI = 0;
    final double lowerD = 0;

    final double upperP = 0.2; //0.0014182
    final double upperI = 0;
    final double upperD = 0;

    final double lowerArmMin = -28; //-17
    final double lowerArmMax = 48; //45
    final double upperArmMin = 3; //10
    final double upperArmMax = 180; //165

    final double lowerArmTolerance = 3;
    final double upperArmTolerance = 3;
    boolean currentStopFlag = false;
    Timer currentTimer = new Timer();
    /* 
     * PRESETS:
     * Cone/cube ground pickup (0.58, -0.17), (0.56, -0.12) X
     * Pickup + open () X
     * Cone upright ground (0.51, -0.05) X
     * Mid cone/cube placing (1.03, 0.87), (1.09, 0.48) X
     * high cone/cube placing (1.43, 1.23), (1.5, 0.77) X
     * Low placing (0.59, 0.2), () X
     * Substation () X
     * Start ()
     */
    public static enum Preset {
        Ground,
        Pickup,
        UprightConeGround,
        MidPlacing,
        HighPlacing,
        LowPlacing,
        Travel,
        Start,
        Substation;
    }

    private boolean isInCubeMode = false;

    public double conePickupX = 0.319366;
    public double conePickupY = 0.166689;
    private final Map<Preset, ArmState> coneMap =
    new EnumMap<>(Map.ofEntries(
        Map.entry(Preset.Ground, ArmState.fromEndEffector(0.58, -0.12)),
        Map.entry(Preset.Pickup, ArmState.fromEndEffector(conePickupX, conePickupY)),
        Map.entry(Preset.UprightConeGround, ArmState.fromEndEffector(0.51, -0.05)),
        Map.entry(Preset.MidPlacing, ArmState.fromEndEffector(1.082306, 0.864651)),
        Map.entry(Preset.LowPlacing, ArmState.fromEndEffector(0.59, 0.2)),
        Map.entry(Preset.Travel, ArmState.fromEndEffector(conePickupX, conePickupY)),
        Map.entry(Preset.HighPlacing, ArmState.fromEndEffector(1.475807, 1.171830))
    ));

    private final Map<Preset, ArmState> cubeMap =
    new EnumMap<>(Map.ofEntries(
        Map.entry(Preset.Ground, ArmState.fromEndEffector(0.56, -0.12)),
        Map.entry(Preset.Pickup, ArmState.fromEndEffector(0.19, 0.12)),
        Map.entry(Preset.MidPlacing, ArmState.fromEndEffector(1.09, 0.48)),
        Map.entry(Preset.HighPlacing, ArmState.fromEndEffector(1.5, 0.77)),
        Map.entry(Preset.Start, ArmState.fromEndEffector(0.19, 0.12)),
        Map.entry(Preset.Travel, ArmState.fromEndEffector(0.272431, 0.269070))
    ));

    public ArmSubsystem(Resolver lowerEncoder,Resolver upperEncoder) {
        fixPresetMaps();
        setupNetworkTables();

        lowerArmMotor2.follow(lowerArmMotor1);
        upperArmMotor2.follow(upperArmMotor1);
        this.lowerEncoder = lowerEncoder;
        this.upperEncoder = upperEncoder;
        lowerArmMotor1.setIdleMode(IdleMode.kBrake);
        lowerArmMotor2.setIdleMode(IdleMode.kBrake);
        upperArmMotor1.setIdleMode(IdleMode.kBrake);
        upperArmMotor2.setIdleMode(IdleMode.kBrake);
        lowerArmMotor1.setSmartCurrentLimit(60);
        lowerArmMotor2.setSmartCurrentLimit(60);
        upperArmMotor1.setSmartCurrentLimit(60);
        upperArmMotor2.setSmartCurrentLimit(60);

        lowerArmMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
		lowerArmMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
		lowerArmMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        lowerArmMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
		lowerArmMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
		lowerArmMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        upperArmMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
		upperArmMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
		upperArmMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        upperArmMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
		upperArmMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
		upperArmMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);


        lowerArmMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        lowerArmMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        upperArmMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        upperArmMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    }

    @Override
    public void periodic() {


        // if (Units.metersToInches(getEndEffectorPosition().getX()) >= 30){
        //     lowerArmVoltage = Math.min()
        // }
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

        /* Current Stop */
        // if ((lowerArmMotor1.getOutputCurrent() >= 25 || upperArmMotor1.getOutputCurrent() >= 25)){
        //     if (!currentStopFlag){
        //         currentStopFlag = true;
        //         currentTimer.reset();
        //         currentTimer.start();
        //     }
        //     if (currentTimer.hasElapsed(0.15)){
        //         lowerArmVoltage = 0;
        //         upperArmVoltage = 0;
        //         new ArmStopCommand(this).schedule();
        //     }
            
        // }
        // else{
        //     currentStopFlag = false;
        // }
        // System.out.println(getEndEffectorPosition());
        lowerArmMotor1.setVoltage(lowerArmVoltage);
        upperArmMotor1.setVoltage(upperArmVoltage);
        

        /* NetworkTables */
        updateNetworkTables();
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

    public boolean getIsInCubeMode() {
        return isInCubeMode;
    }

    public Translation2d getEndEffectorPosition(){
        ArmKinematics kinematics = new ArmKinematics(Constants.PhysicalDimensions.kLowerArmLength, Constants.PhysicalDimensions.kUpperArmLength);
        kinematics.setLowerAngle(Math.toRadians(getLowerPosition()));
        kinematics.setUpperAngle(Math.toRadians(getUpperPosition()));
        kinematics.fowardKinematics();
        Translation2d point = new Translation2d(kinematics.getX(), kinematics.getY());
        return point;
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
    }

    public void setIsInCubeMode(boolean isInCubeMode) {
        this.isInCubeMode = isInCubeMode;
    }

    public double calcLowerFFVoltage(double velocity, double accel){
        return lowerFF.calculate(velocity, accel);
    }

    public double calcUpperFFVoltage(double velocity, double accel){
        return upperFF.calculate(velocity, accel);
    }
    
    public double calcLowerFFVoltage(double velocity){
        return lowerFF.calculate(velocity);
    }

    public double calcUpperFFVoltage(double velocity){
        return upperFF.calculate(velocity);
    }

    public Command createArmProfileCommand(double posLowerDegrees, double posUpperDegrees) {
        ProfiledPIDController lowerController = createControllerLower();
        Command commandLower = new ProfiledPIDCommand(lowerController, () -> getLowerPosition(), 
            new TrapezoidProfile.State(posLowerDegrees, 0), 
                (pidV, trapState) -> setLowerVoltage(-(calcLowerFFVoltage(pidV + trapState.velocity))))
            .until(() -> lowerController.atGoal());
        
        ProfiledPIDController upperController = createControllerUpper();
        Command commandUpper = new ProfiledPIDCommand(upperController, () -> getUpperPosition(), 
            new TrapezoidProfile.State(posUpperDegrees, 0), 
                (pidV, trapState) -> setUpperVoltage(-(calcUpperFFVoltage(pidV + trapState.velocity))))
            .until(() -> upperController.atGoal());
        
        ParallelCommandGroup group = new ParallelCommandGroup(commandLower, commandUpper);
        group.addRequirements(this);
        return group;
    }

    public Command createArmProfileCommand(Preset preset) {
        return new InstantCommand(
            () -> {
                Map<Preset, ArmState> presetMap = isInCubeMode ? cubeMap : coneMap;
                createArmProfileCommand(presetMap.get(preset).theta1Degrees, presetMap.get(preset).theta2Degrees)
                    .schedule();
            }
        );
    }

    public Command createEndEffectorProfileCommand(Preset preset) {
        return new InstantCommand(
            () -> {
                Map<Preset, ArmState> presetMap = isInCubeMode ? cubeMap : coneMap;
                createEndEffectorProfileCommand(presetMap.get(preset).x, presetMap.get(preset).y)
                    .schedule();
            }
        );
    }
    public Command createEndEffectorProfileCommandNoInstant(Preset preset) {
                Map<Preset, ArmState> presetMap = isInCubeMode ? cubeMap : coneMap;
                return createEndEffectorProfileCommand(presetMap.get(preset).x, presetMap.get(preset).y);
    }

    public Command createEndEffectorProfileCommand(double x, double y) {
        ProfiledPIDController controller = createControllerEndEffector();
        Translation2d goalPose = new Translation2d(x, y);
        ArmMath math = new ArmMath(Constants.PhysicalDimensions.kLowerArmLength, Constants.PhysicalDimensions.kUpperArmLength);

        return new ProfiledPIDCommand(
            controller,
            () -> -goalPose.getDistance(getEndEffectorPosition()),
            new TrapezoidProfile.State(0, 0),
            (pid, nextState) -> {
                double speed = nextState.velocity;

                Translation2d diff = goalPose.minus(getEndEffectorPosition());
                if (diff.getNorm() > 1e-5) {
                    diff = diff.times((speed + pid) / diff.getNorm());
                }
                math.setTheta1(Math.toRadians(getLowerPosition()));
                math.setTheta2(Math.toRadians(getUpperPosition()));
                math.setVx(diff.getX());
                math.setVy(diff.getY());
                math.inverseKinematics();
                setLowerVoltage(-calcLowerFFVoltage(Math.toDegrees(math.getOmega1())));
                setUpperVoltage(-calcUpperFFVoltage(Math.toDegrees(math.getOmega2())));
            },
            this // add ArmSubsystem requirement
        ).until(
            () -> controller.atGoal()
        );
    }

    // public Command createTripleMoveCommand(double lowerPos, double upperPos) {
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

    private ProfiledPIDController createControllerLower(){
        ProfiledPIDController controller = new ProfiledPIDController(lowerP,lowerI,lowerD, new TrapezoidProfile.Constraints(lowerArmMaxSpeed, lowerArmMaxAccel));
        controller.setTolerance(lowerArmTolerance);
        return controller;
    }

    private ProfiledPIDController createControllerUpper(){
        ProfiledPIDController controller = new ProfiledPIDController(upperP,upperI,upperD, new TrapezoidProfile.Constraints(upperArmMaxSpeed, upperArmMaxAccel));
        controller.setTolerance(upperArmTolerance);
        return controller;
    }

    private ProfiledPIDController createControllerEndEffector() {
        ProfiledPIDController controller = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(2.2, 0.7));
        controller.setTolerance(0.02);
        return controller;
    }

    /*
     * This method copies all presets that aren't in both maps so that they are in both maps.
     */
    private void fixPresetMaps() {
        for (Entry<Preset, ArmState> entry: cubeMap.entrySet()) {
            Preset preset = entry.getKey();
            ArmState value = entry.getValue();

            if (!coneMap.containsKey(preset)) {
                coneMap.put(preset, value);
            }
        }

        for (Entry<Preset, ArmState> entry: coneMap.entrySet()) {
            Preset preset = entry.getKey();
            ArmState value = entry.getValue();

            if (!cubeMap.containsKey(preset)) {
                cubeMap.put(preset, value);
            }
        }
    }

    /* NetworkTables */

    NetworkTableInstance nt;
    NetworkTable table;
    DoublePublisher xPub, yPub;
    DoublePublisher xInchesPub, yInchesPub;
    DoublePublisher theta1Pub, theta2Pub;
    BooleanPublisher isInCubeModePub;

    private void setupNetworkTables() {
        nt = NetworkTableInstance.getDefault();
        table = nt.getTable("armSubsystem");
        xPub = table.getDoubleTopic("x").publish();
        yPub = table.getDoubleTopic("y").publish();
        xInchesPub = table.getDoubleTopic("xInches").publish();
        yInchesPub = table.getDoubleTopic("yInches").publish();
        theta1Pub = table.getDoubleTopic("theta1").publish();
        theta2Pub = table.getDoubleTopic("theta2").publish();
        isInCubeModePub = table.getBooleanTopic("isInCubeMode").publish();
    }

    private void updateNetworkTables() {
        Translation2d endEffectorPos = getEndEffectorPosition();
        double x = endEffectorPos.getX();
        double y = endEffectorPos.getY();

        xPub.set(x);
        yPub.set(y);
        xInchesPub.set(Units.metersToInches(x));
        yInchesPub.set(Units.metersToInches(y));
        theta1Pub.set(getLowerPosition());
        theta2Pub.set(getUpperPosition());
        isInCubeModePub.set(isInCubeMode);
    }
}