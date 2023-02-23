package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmState {
    public double theta1;
    public double theta2;
    public ArmState(double theta1Radians, double theta2Radians){
        this.theta1 = theta1Radians;
        this.theta2 = theta2Radians;
    }
    public static ArmState fromEndEffector(double x, double y){
        ArmKinematics math = new ArmKinematics(Units.inchesToMeters(39), Units.inchesToMeters(35.4)); 
        math.setX(x);
        math.setY(y);
        math.inverseKinematics();
        return new ArmState(math.getLowerAngle(), math.getUpperAngle());
    }
}
