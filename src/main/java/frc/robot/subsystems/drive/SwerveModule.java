// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.sensors.Resolver;
import frc.robot.subsystems.drive.PivotConfig.PivotId;

public class SwerveModule {
	private static final double kWheelRadius = 0.054; //0.0508 

	private final CANSparkMax driveMotor;
	private final CANSparkMax steeringMotor;

	private RelativeEncoder driveEncoder;
	public Resolver steeringEncoder;

	private final PIDController drivePIDController = new PIDController(2.2019, 0.0, 0.0);

	private final PIDController turningPIDController = new PIDController(0.725, 0.0, 0.005);

	private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.20979, 3.0986); //0.19263, 0.016349
	private PivotConfig cfg;
	public double targetSpeedAuto;
	public SwerveModule(PivotConfig cfg) {
		this.cfg = cfg;
		driveMotor = new CANSparkMax(cfg.getDriveChannel(), MotorType.kBrushless);
		steeringMotor = new CANSparkMax(cfg.getSteerChannel(), MotorType.kBrushless);
		// driveMotor.setSmartCurrentLimit(60);
		// steeringMotor.setSmartCurrentLimit(40);
		// driveMotor.setIdleMode(IdleMode.kCoast);
		// driveMotor.burnFlash();
		// steeringMotor.burnFlash();
		driveEncoder = driveMotor.getEncoder();
		steeringEncoder = new Resolver(cfg.getResolverChannel(), cfg.getMinvoltage(), cfg.getMaxvoltage(),
				cfg.getOffset(), cfg.isReverseAngle());
		// System.out.println("Max: " + cfg.getOffset());
		driveMotor.setInverted(cfg.isReverseDrive());
		steeringMotor.setInverted(cfg.isReverseSteer());
		turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
		steeringMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
		steeringMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
		steeringMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
		steeringMotor.burnFlash();
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(getVelocity(), new Rotation2d(steeringEncoder.get()));
	}

	public void resetEncoder() {
		driveEncoder.setPosition(0);
	}

	public double getPositionX() {
		return driveEncoder.getPosition() * kWheelRadius * 0.10472 * 0.12 * Math.sin(steeringEncoder.get());
	}
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			((-driveEncoder.getPosition() / 7.73) * kWheelRadius * 2 * Math.PI), new Rotation2d(steeringEncoder.get()));
	  }
	public double getPositionY() {
		return (driveEncoder.getPosition()) * kWheelRadius * 0.10472 * 0.12 * Math.cos(steeringEncoder.get());
	}


	public double getVelocity() {
		//return driveEncoder.getVelocity();
		
		return ((-driveEncoder.getVelocity() / 7.73) / 60) * 2 * Math.PI * kWheelRadius;
	}
	public RelativeEncoder getDriveEncoder(){
		return driveEncoder;
	}
	public Resolver getSteeringEncoder(){
		return steeringEncoder;
	}
	public double echo(double val) {
		return val;
	}

	public SwerveModuleState desiredState = null;

	public void setDesiredState(SwerveModuleState state) {
		desiredState = state;
		// if (cfg.getName() == PivotId.FL){
		// 	System.out.format("%.2f, %.2f\n", desiredState.angle.getDegrees(), steeringEncoder.getD());
		// }
		double dAngle = state.angle.getDegrees() - steeringEncoder.getD();
		double dAngleAbs = Math.abs(dAngle) % 360;
		boolean flipDrive = (90.0 <= dAngleAbs) && (dAngleAbs <= 270.0);
		double sin = Math.sin(dAngle * Math.PI / 180.0);
		sin = (flipDrive) ? -sin : sin;
		double turnOutput = turningPIDController.calculate(sin, 0);

		final double targetSpeed = flipDrive ? state.speedMetersPerSecond : -state.speedMetersPerSecond;

		if (Math.abs(targetSpeed) < 0.1) {
			turnOutput = 0;
		}

		driveMotor.set(targetSpeed);
		steeringMotor.set(turnOutput);
	}

	public void setDesiredStateAuto(SwerveModuleState state) {
		desiredState = state;

		double dAngle = state.angle.getDegrees() - steeringEncoder.getD();
		double dAngleAbs = Math.abs(dAngle) % 360;
		boolean flipDrive = (90.0 <= dAngleAbs) && (dAngleAbs <= 270.0);
		double sin = Math.sin(dAngle * Math.PI / 180.0);
		sin = (flipDrive) ? -sin : sin;
		double turnOutput = turningPIDController.calculate(sin, 0);

		final double targetSpeed = flipDrive ? -state.speedMetersPerSecond : state.speedMetersPerSecond;
		targetSpeedAuto=targetSpeed;
		if (Math.abs(targetSpeed) < 0.1) {
			turnOutput = 0;
		}

		double driveFeedForward = -driveFeedforward.calculate(targetSpeed);
		double driveOutput = -drivePIDController.calculate(getVelocity(), targetSpeed);

		driveMotor.setVoltage(driveFeedForward + driveOutput);
		steeringMotor.set(turnOutput);
	}
	public void setAngleD(double angle) {
		double dAngle = angle - steeringEncoder.getD();
		double dAngleAbs = Math.abs(dAngle) % 360;
		boolean flipDrive = (90.0 <= dAngleAbs) && (dAngleAbs <= 270.0);
		double sin = Math.sin(dAngle * Math.PI / 180.0);
		sin = (flipDrive) ? -sin : sin;
		double turnOutput = turningPIDController.calculate(sin, 0);

		steeringMotor.set(turnOutput);
	}

	public void setSpeed(double speed) {
		driveMotor.set(speed);
	}
}
