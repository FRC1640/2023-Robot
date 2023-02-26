package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.arm.ArmSubsystem;

public class DashboardInit {
    public DashboardInit(Gyro gyro, ArmSubsystem armSubsystem){
        ShuffleboardTab testTab = Shuffleboard.getTab("Testing");
        testTab.addDouble("Arm position", () -> System.currentTimeMillis()).withWidget(BuiltInWidgets.kGraph).withSize(5, 5).withPosition(0, 0);
        // testTab.add("Gyro", gyro.getGyro()).withSize(5, 5).withPosition(0, 0).withWidget(BuiltInWidgets.kGyro);
    }
}