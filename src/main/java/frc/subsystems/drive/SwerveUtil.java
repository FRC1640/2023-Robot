package frc.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

public class SwerveUtil {
    public static Vector<N2> applyDeadzone(double x, double y, double lowerDB, double upperDB){
        upperDB = 1 - upperDB;

        double r = Math.sqrt((x*x) + (y*y));
        if (r < lowerDB){
            return VecBuilder.fill(0, 0);
        } else if (r > upperDB){
            return VecBuilder.fill(x, y).div(r);
        } else {
            return VecBuilder.fill(x, y).div(r).times((r - lowerDB)/(upperDB - lowerDB));
        }
    }

    public static Vector<N2> applySensitivity(double x, double y, double n){
        double r = Math.sqrt((x*x) + (y*y));
        return VecBuilder.fill(x, y).div(r).times(Math.pow(r, n));
    }
}
