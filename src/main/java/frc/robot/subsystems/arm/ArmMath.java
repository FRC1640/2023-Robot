package frc.robot.subsystems.arm;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N2;

public class ArmMath {
    double lowerLength;
    double upperLength;

    double theta1;
    double theta2;

    double omega1;
    double omega2;

    double x;
    double y;

    double xPrime;
    double yPrime;

    MatBuilder<>, Nat<N2>> matBuilder = new MatBuilder<>(Nat.N2(), Nat.N2());

    public ArmMath(double lowerLength, double upperLength){
        this.lowerLength = lowerLength;
        this.upperLength = upperLength;
    }
    public void kinematicsThing(){
        x = lowerLength * Math.sin(theta1) + upperLength * Math.sin(theta1 + theta2);
        y = lowerLength * Math.cos(theta1) + upperLength * Math.cos(theta1 + theta2);
        xPrime =lowerLength * Math.cos(theta1) * omega1 + upperLength * Math.cos(theta1 + theta2) * (omega1 + omega2);
        yPrime =lowerLength * Math.sin(theta1) * omega1 - upperLength * Math.sin(theta1 + theta2) * (omega1 + omega2);
        
    }
    public void setTheta1(double theta1){
        this.theta1 = theta1;
    }
    public void setTheta2(double theta2){
        this.theta2 = theta2;
    }
    public void setOmega1(double omega1){
        this.omega1 = omega1;
    }
    public void setOmega2(double omega2){
        this.omega2 = omega2;
    }
}
