package frc.robot.subsystems.arm;

import com.fasterxml.jackson.databind.PropertyNamingStrategies.UpperCamelCaseStrategy;

public class ArmKinematics {
    /*
     * ASSUMPTIONS:
     * Theta1 = 0 when lower arm points straight up
     * Theta1 and theta2 increase torwards the front of the robot 
     * Theta2 = 0 when arms are parallel 
     * Using radians
     */
    double lowerLength;
    double upperLength;
    double x;
    double y;
    double theta1;
    double theta2;

    public ArmKinematics(double lowerLength, double upperLength){
        this.lowerLength = lowerLength;
        this.upperLength = upperLength;
    }
    /*
     * Converts angle to position
     */
    public void fowardKinematics(){
        double x1 = lowerLength * Math.sin(theta1);
        double y1 = lowerLength * Math.cos(theta1);
        // System.out.println("x1: " + x1 + "y1: " + y1);
        x = x1 + (upperLength * Math.sin(theta1 + theta2));
        y = y1 + (upperLength * Math.cos(theta1 + theta2));
    }
    /*
     * Converts position to angle
     */
    public void inverseKinematics(){
        theta2 = Math.acos((x * x + y * y - lowerLength * lowerLength - upperLength * upperLength) / (2 * lowerLength * upperLength));
        System.out.println(theta2);
        theta1 = Math.PI / 2 - Math.atan(y/x) + Math.atan((upperLength * Math.sin(-theta2)) / (lowerLength + upperLength * Math.cos(-theta2)));
        if (Double.isNaN(theta1) || Double.isNaN(theta2)){
            throw new IllegalStateException(String.format("The position %.2f,%.2f is not possible", x,y));
        }
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getLowerAngle(){
        return theta1;
    }
    public double getUpperAngle(){
        return theta2;
    }


    public void setX(double x){
        this.x = x;
    }
    public void setY(double y){
        this.y = y;
    }
    public void setLowerAngle(double theta){
        this.theta1 = theta;
    }
    public void setUpperAngle(double theta){
        this.theta2 = theta;
    }
}
