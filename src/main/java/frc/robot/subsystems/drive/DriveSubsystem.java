// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.stream.Stream;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.drive.PivotConfig.PivotId;

/** Represents a swerve drive style drivetrain. */
public class DriveSubsystem extends SubsystemBase {
  private Gyro gyro;
  
  public static final double kMaxSpeed = 6; // 3 meters per second  
  public static final double kMaxAngularSpeed = 2 * Math.PI; // 1/2 rotation per second  

  public static final double x = Units.inchesToMeters(10.375); // 10.375"
  public static final double y = Units.inchesToMeters(12.375); // 12.375"
  private final Translation2d centerLocation = new Translation2d(0,0);

  private final Translation2d frontLeftLocation = new Translation2d(-x, y);
  private final Translation2d frontRightLocation = new Translation2d(x, y);
  private final Translation2d backLeftLocation = new Translation2d(-x, -y);
  private final Translation2d backRightLocation = new Translation2d(x, -y);

  private final SwerveModule frontLeft = new SwerveModule(PivotConfig.getConfig(PivotId.FL));
  private final SwerveModule frontRight = new SwerveModule(PivotConfig.getConfig(PivotId.FR));
  private final SwerveModule backLeft = new SwerveModule(PivotConfig.getConfig(PivotId.BL));
  private final SwerveModule backRight = new SwerveModule(PivotConfig.getConfig(PivotId.BR));

  public Field2d field = new Field2d();
  
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry;
    public DriveSubsystem(Gyro gyro) {
      this.gyro = gyro;
      odometry = new SwerveDriveOdometry(
          kinematics,
          gyro.getRotation2d(),
          new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              backLeft.getPosition(),
              backRight.getPosition()
          });
    }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
      gyro.getRotation2d(),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      },
      pose
    );
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    driveAxis(xSpeed, ySpeed, rot, fieldRelative, centerLocation);
    //prints
    // System.out.format("%.2f, %.2f, %.2f, %.2f\n", frontLeft.getSteeringEncoder().getD(), frontRight.getSteeringEncoder().getD(), backLeft.getSteeringEncoder().getD(),backRight.getSteeringEncoder().getD());
    // System.out.format("%.2f, %.2f, %.2f, %.2f\n", frontLeft.getDriveEncoder().getVelocity(), frontRight.getDriveEncoder().getVelocity(), backLeft.getDriveEncoder().getVelocity(),backRight.getDriveEncoder().getVelocity());
  }
  public void driveAxis(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Translation2d rotAxis) {
    var swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                new Rotation2d(gyro.getRotation2d().getRadians()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot),
            rotAxis);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeed);
    frontLeft.setDesiredStateAuto(desiredStates[0]);
    frontRight.setDesiredStateAuto(desiredStates[1]);
    backLeft.setDesiredStateAuto(desiredStates[2]);
    backRight.setDesiredStateAuto(desiredStates[3]);
  }
  
  public void pointWheels(double angle) {
    frontLeft.setAngleD(angle);
    frontRight.setAngleD(angle);
    backLeft.setAngleD(angle);
    backRight.setAngleD(angle);
  }

  public void xWheels() {
    frontLeft.setAngleD(45);
    frontRight.setAngleD(-45);
    backLeft.setAngleD(-45);
    backRight.setAngleD(45);
  }

  public void setSpeeds(double speed) {
    frontLeft.setSpeed(speed);
    frontRight.setSpeed(speed);
    backLeft.setSpeed(speed);
    backRight.setSpeed(speed);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
  }
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetEncoders() {
    Stream.of(frontLeft, frontRight, backLeft, backRight).forEach(SwerveModule::resetEncoder);
  }

  @Override
  public void periodic() {
    updateOdometry();
  }
}