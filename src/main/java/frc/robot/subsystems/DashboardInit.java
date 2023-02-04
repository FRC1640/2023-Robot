package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import frc.robot.sensors.Gyro;

public class DashboardInit {
    public DashboardInit(Gyro gyro){
        ShuffleboardTab testTab = Shuffleboard.getTab("Testing");
        testTab.add("Gyro", gyro.getGyro()).withSize(5, 5).withPosition(0, 0).withWidget(BuiltInWidgets.kGyro);
    }
    
}
