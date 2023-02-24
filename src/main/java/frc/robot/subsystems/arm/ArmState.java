package frc.robot.subsystems.arm;

import frc.robot.Constants;

public class ArmState {
    public double theta1Degrees;
    public double theta2Degrees;
    public ArmState(double theta1Degrees, double theta2Degrees){
        this.theta1Degrees = theta1Degrees;
        this.theta2Degrees = theta2Degrees;
    }
    public static ArmState fromEndEffector(double x, double y){
        ArmKinematics math = new ArmKinematics(Constants.PhysicalDimensions.kLowerArmLength, Constants.PhysicalDimensions.kUpperArmLength); 
        math.setX(x);
        math.setY(y);
        math.inverseKinematics();
        return new ArmState(Math.toDegrees(math.getLowerAngle()), Math.toDegrees(math.getUpperAngle()));
    }
}
