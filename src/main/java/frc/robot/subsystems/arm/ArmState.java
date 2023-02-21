package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmState {
    double theta1;
    double theta2;
    public ArmState(double theta1, double theta2){
        this.theta1 = theta1;
        this.theta2 = theta2;
    }
    public static ArmState fromEndEffector(double x, double y){
        ArmKinematics math = new ArmKinematics(Units.inchesToMeters(39), Units.inchesToMeters(35.4)); 
        math.setX(x);
        math.setY(y);
        math.inverseKinematics();
        return new ArmState(math.getLowerAngle(), math.getUpperAngle());
    }
}
