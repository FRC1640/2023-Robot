package frc.robot.subsystems.arm;

import edu.wpi.first.math.Matrix;

import org.ejml.data.SingularMatrixException;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.XboxController;

public class ArmMath {
    double lowerLength;
    double upperLength;

    double theta1;
    double theta2;

    double omega1;
    double omega2;

    double vx;
    double vy;

    double xPrime;
    double yPrime;

    MatBuilder<N2, N2> matBuilder = new MatBuilder<>(Nat.N2(), Nat.N2());
    
    public ArmMath(double lowerLength, double upperLength){
        this.lowerLength = lowerLength;
        this.upperLength = upperLength;
    }
    
    // public static void main(String[] args){
    //     ArmMath math = new ArmMath(1, 1);
    //     // ArmKinematics kinematics = new ArmKinematics(lowerLength, upperLength);
    //     math.setTheta1(0);
    //     math.setTheta2(0);
    //     math.setVx(0);
    //     math.setVy(1);

    //     try {
    //         math.inverseKinematics();
    //         math.omega1 = math.getOmega1();
    //         math.omega2 = math.getOmega2();
    //     }
    //     catch(SingularMatrixException exception){
    //         math.setOmega1(0);
    //         math.setOmega2(0);
    //     }
        
    //     System.out.println("Omega1: " + math.getOmega1() + "Omega2: " + math.getOmega2());
    //     math.forwardKinematics();
    //     System.out.println("VX: " + math.vx + "VY: " + math.vy);
    // }
    public Matrix<N2,N2> calcJacobian(){
        double c1 = Math.cos(theta1);
        double c2 = Math.cos(theta1 + theta2);
        double s1 = Math.sin(theta1);
        double s2 = Math.sin(theta1 + theta2);
        
        return matBuilder.fill(lowerLength * c1 + upperLength * c2, upperLength * c2,
            -lowerLength * s1 - upperLength * s2, -upperLength * s2);
    }
    public void inverseKinematics() throws SingularMatrixException{
        Matrix<N2, N2> J = calcJacobian();
        
        Vector<N2> v = VecBuilder.fill(vx, vy);
        Vector<N2> omega = new Vector<>(J.solve(v.extractColumnVector(0)));
        setOmega1(omega.get(0, 0));
        setOmega2(omega.get(1, 0));
    }
    public void forwardKinematics(){
        Matrix<N2, N2> J = calcJacobian();
        Vector<N2> v = new Vector<>(J.times(VecBuilder.fill(omega1, omega2)));
        vx = v.get(0, 0);
        vy = v.get(1, 0);
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
    public void setVx(double vx){
        this.vx = vx;
    }
    public void setVy(double vy){
        this.vy = vy;
    }



    public double getTheta1(){
        return theta1;
    }
    public double getTheta2(){
        return theta2;
    }
    public double getOmega1(){
        return omega1;
    }
    public double getOmega2(){
        return omega2;
    }
    public double getVx(){
        return vx;
    }
    public double getVy(){
        return vy;
    }
}
