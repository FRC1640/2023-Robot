package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.Resolver;

public class WristSubsystem extends SubsystemBase{
    PIDController wristController = new PIDController(1, 0, 0);
    CANSparkMax wristMotor = new CANSparkMax(12, MotorType.kBrushless);
    final double wristMax =2.6; 
    final double wristMin = 0.5;

    private Resolver wristEncoder;



    public WristSubsystem(Resolver wristEncoder){
        this.wristEncoder = wristEncoder;
        setupNetworkTables();        
    }
    public double getWristPosition(){
        return wristEncoder.getV();
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
        double speed = wristController.calculate(getWristPosition(), position);
        //System.out.println(speed);
        runWrist(-speed);

    }
    @Override
    public void periodic() {
        // System.out.println("Wrist: " + getWristPosition());
        updateNetworkTables();
    }

    /* Network Table */
    NetworkTableInstance nt;
    NetworkTable table;
    DoublePublisher anglPub;

    private void setupNetworkTables() {
        nt = NetworkTableInstance.getDefault();
        table = nt.getTable("wristSubsystem");
        anglPub = table.getDoubleTopic("resolverVoltage").publish();    
    }
    private void updateNetworkTables() {
        double angle = getWristPosition();

        anglPub.set(angle);
    } 
}
