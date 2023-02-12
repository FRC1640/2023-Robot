package frc.robot.subsystems.arm;

public class ArmState {
    double theta1;
    double theta2;
    public ArmState(double theta1, double theta2){
        this.theta1 = theta1;
        this.theta2 = theta2;
    }
    public static ArmState FromEndEffector(double x, double y){
        ArmMath math = new ArmMath(0, 0); //TODO: ADD LENGTHS
        math.setX(x);
        math.setY(y);
        math.inverseKinematics();
        return new ArmState(math.getLowerAngle(), math.getUpperAngle());
    }
}
