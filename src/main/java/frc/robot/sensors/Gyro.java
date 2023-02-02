package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;

public class Gyro {
    private double offset;
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    public void resetGyro() {
        offset = 0;
        DataLogManager.log("Reset gyro");
        gyro.reset();
      }
    public AHRS getGyro(){
      return gyro;
    }
    public Rotation2d getGyroAngle() {
      return new Rotation2d(gyro.getRotation2d().getRadians() - Math.toRadians(offset));
    }
    public double getGyroAngleDegrees(){
      return new Rotation2d(gyro.getRotation2d().getRadians() - Math.toRadians(offset)).getDegrees();
    }
    public double getGyroPitch(){
      return gyro.getPitch();
    }
    public Rotation2d getRotation2d(){
      return new Rotation2d(gyro.getRotation2d().getRadians() - Math.toRadians(offset));
    }
    public Rotation2d getRaw(){
      return gyro.getRotation2d();
    }
    public void setOffset(double value) {
      offset = value;
    }
  
    public double getOffset() {
      return offset;
    }
}
