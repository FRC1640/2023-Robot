package frc.robot.subsystems.arm;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
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

    MatBuilder<N2, N2> matBuilder = new MatBuilder<>(Nat.N2(), Nat.N2());
    Matrix<N2,N2> J = matBuilder.fill(1, 0, 0, 1);
    Vector<N2> v = VecBuilder.fill(1,0);
    Vector<N2> omega = new Vector<>(J.solve(v.extractColumnVector(0)));
    public ArmMath(double lowerLength, double upperLength){
        this.lowerLength = lowerLength;
        this.upperLength = upperLength;
    }
    // public Matrix<N2,N2> calcJacobian(){
    //     MatBuilder<N2, N2> matBuilder = new MatBuilder<>(Nat.N2(), Nat.N2());
    //     double c1 = Math.cos(theta1);
    //     double c2 = Math.cos(theta1 + theta2);
    //     double s1 = Math.sin(theta1);
    //     double s2 = Math.sin(theta1 + theta2);
    //     Matrixm = Matrix<N2,N2> m = matBuilder.fill(lowerLength * c1 + upperLength * c2, upperLength * c2,
    //     -lowerLength * s1 - upperLength * s2, -upperLength * s2);
    //     return 
        
    // }
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
