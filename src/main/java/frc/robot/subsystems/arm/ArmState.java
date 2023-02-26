package frc.robot.subsystems.arm;

import frc.robot.Constants;

public class ArmState {
    public final double theta1Degrees;
    public final double theta2Degrees;
    public final double x;
    public final double y;
    public ArmState(double theta1Degrees, double theta2Degrees){
        this.theta1Degrees = theta1Degrees;
        this.theta2Degrees = theta2Degrees;
        ArmKinematics math = new ArmKinematics(Constants.PhysicalDimensions.kLowerArmLength, Constants.PhysicalDimensions.kUpperArmLength); 
        math.setLowerAngle(Math.toRadians(theta1Degrees));
        math.setUpperAngle(Math.toRadians(theta2Degrees));
        math.fowardKinematics();
        this.x = math.getX();
        this.y = math.getY();
    }
    public static ArmState fromEndEffector(double x, double y){
        ArmKinematics math = new ArmKinematics(Constants.PhysicalDimensions.kLowerArmLength, Constants.PhysicalDimensions.kUpperArmLength); 
        math.setX(x);
        math.setY(y);
        math.inverseKinematics();
        return new ArmState(Math.toDegrees(math.getLowerAngle()), Math.toDegrees(math.getUpperAngle()));
    }
}
