// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.stream.Stream;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.sensors.Gyro;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.drive.PivotConfig.PivotId;
import frc.robot.utilities.ArrayToPose;
import frc.robot.utilities.MathUtils;

/** Represents a swerve drive style drivetrain. */
public class DriveSubsystem extends SubsystemBase {
  private Gyro gyro;
  
  public static final double kMaxSpeed = 6; // 3 meters per second  
  public static final double kMaxAngularSpeed = 2 * Math.PI; // 1/2 rotation per second  

  public static final double y = Units.inchesToMeters(10.375); // 10.375"
  public static final double x = Units.inchesToMeters(12.375); // 12.375"

  private final Translation2d frontLeftLocation = new Translation2d(x, y);
  private final Translation2d frontRightLocation = new Translation2d(x, -y);
  private final Translation2d backLeftLocation = new Translation2d(-x, y);
  private final Translation2d backRightLocation = new Translation2d(-x, -y);

  private final SwerveModule frontLeft = new SwerveModule(PivotConfig.getConfig(PivotId.FL));
  private final SwerveModule frontRight = new SwerveModule(PivotConfig.getConfig(PivotId.FR));
  private final SwerveModule backLeft = new SwerveModule(PivotConfig.getConfig(PivotId.BL));
  private final SwerveModule backRight = new SwerveModule(PivotConfig.getConfig(PivotId.BR));

  public double translationStdDevCoefficient = 0.3;
  private final double rotationStdDevCoefficient = 0.9;
  public static final Transform3d limelightRobotToCamera = new Transform3d(
    new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(34.25)),
    new Rotation3d(0, Math.toRadians(15), Math.PI));
  // private final Field2d field2d = new Field2d();

  // public Field2d field = new Field2d();

  Limelight limelight;
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDrivePoseEstimator odometry;
    public DriveSubsystem(Gyro gyro, Limelight limelight) {
      this.gyro = gyro;
      this.limelight = limelight;
      setupNetworkTables();

      ShuffleboardTab fieldTab = Shuffleboard.getTab("Field");
      // fieldTab.add("Field", field2d);
      
      odometry = new SwerveDrivePoseEstimator(
          kinematics,
          gyro.getRotation2d(),
          new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              backLeft.getPosition(),
              backRight.getPosition()
          },
          new Pose2d(0, 0, Rotation2d.fromDegrees(0))
      );
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
    var swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                new Rotation2d(gyro.getRotation2d().getRadians()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
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

  public SwerveDriveKinematics createKinematics(){
    return new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
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

  public void print(){
    // System.out.println("Pos: " + frontRight.getPosition());
    // System.out.println("Gyro: " + gyro.getGyroPitch());
    // System.out.format("%.2f, %.2f | %.2f, %.1f, %.2f, %.1f, %.2f, %.1f, %.2f, %.1f | %.1f\n", 
    // odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), 
    // frontLeft.getPosition().distanceMeters, frontLeft.getPosition().angle.getDegrees(),
    // frontRight.getPosition().distanceMeters, frontRight.getPosition().angle.getDegrees(),
    // backLeft.getPosition().distanceMeters, backLeft.getPosition().angle.getDegrees(),
    // backRight.getPosition().distanceMeters, backRight.getPosition().angle.getDegrees(),
    // gyro.getRotation2d().getDegrees());
  }
  /** Updates the field relative position of the robot. */
  private boolean isValidPose(Pose3d pose) {
    boolean isWithinField = MathUtils.isInRange(pose.getY(), -5, FieldConstants.fieldWidth + 5)
            && MathUtils.isInRange(pose.getX(), -5, FieldConstants.fieldLength + 5)
            && MathUtils.isInRange(pose.getZ(), 0, 5);

    boolean isNearRobot = getPose()
                    .getTranslation()
                    .getDistance(pose.getTranslation().toTranslation2d())
            < 1.4;

    return isWithinField && isNearRobot;
}
  private Matrix<N3, N1> calculateVisionStdDevs(double distance) {
    var translationStdDev = translationStdDevCoefficient * Math.pow(distance, 2);
    var rotationStdDev = rotationStdDevCoefficient * Math.pow(distance, 2);

    return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
}

  public void updateOdometry() {

    double[] poseArray = limelight.getBotPose();
    if (limelight.getAprilTagID() >= 1 && limelight.getAprilTagID() <= 8){
      
      Pose3d pose = ArrayToPose.convert(poseArray).transformBy(limelightRobotToCamera.inverse());
      var aprilTagPose = FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(limelight.getAprilTagID());
      var distanceFromPrimaryTag = aprilTagPose.get().getTranslation().getDistance(pose.getTranslation());
      if (isValidPose(pose)){
        Pose2d pose2d = new Pose2d(new Translation2d(pose.getX(), pose.getY()), new Rotation2d(pose.getRotation().getZ()));
        odometry.addVisionMeasurement(pose2d, Timer.getFPGATimestamp() - poseArray[6] / 1000.0, calculateVisionStdDevs(distanceFromPrimaryTag));
        
      }
      // System.out.println("AprilTagPose: " + aprilTagPose.get().getTranslation());
      // System.out.println("Pose:" + pose.getTranslation());
      System.out.println("Distance: " + distanceFromPrimaryTag);
      
    }
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
    


  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void resetEncoders() {
    Stream.of(frontLeft, frontRight, backLeft, backRight).forEach(SwerveModule::resetEncoder);
  }

  @Override
  public void periodic() {
    updateOdometry();
    updateNetworkTables();
    // field.setRobotPose(getPose());
  }
  NetworkTableInstance nt;
  NetworkTable odometryTable, limelightTable;
  DoublePublisher xPub, yPub; 

  private void setupNetworkTables() {
      nt = NetworkTableInstance.getDefault();
      odometryTable = nt.getTable("odometry");
      xPub = odometryTable.getDoubleTopic("x").publish();
      yPub = odometryTable.getDoubleTopic("y").publish();
  }

  private void updateNetworkTables() {
      double x = getPose().getX();
      double y = getPose().getY();

      xPub.set(x);
      yPub.set(y);
  }
}
