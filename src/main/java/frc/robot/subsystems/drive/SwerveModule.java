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
import frc.robot.subsystems.drive.PivotConfig.PivotId;

public class SwerveModule {
	private static final double kWheelRadius = 0.054; //0.0508 

	private final CANSparkMax driveMotor;
	private final CANSparkMax turningMotor;

	private RelativeEncoder driveEncoder;
	public Resolver turningEncoder;

	private final PIDController drivePIDController = new PIDController(2.2019, 0.0, 0.0);

	private final PIDController turningPIDController = new PIDController(0.725, 0.0, 0.005);

	private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.20979, 3.0986); //0.19263, 0.016349
	private PivotConfig cfg;
	public double targetSpeedAuto;
	public SwerveModule(PivotConfig cfg) {
		this.cfg = cfg;
		driveMotor = new CANSparkMax(cfg.getDriveChannel(), MotorType.kBrushless);
		turningMotor = new CANSparkMax(cfg.getSteerChannel(), MotorType.kBrushless);
		driveMotor.setIdleMode(IdleMode.kBrake);
		driveEncoder = driveMotor.getEncoder();
		turningEncoder = new Resolver(cfg.getResolverChannel(), cfg.getMinVoltage(), cfg.getMaxVoltage(),
				cfg.getOffset(), cfg.isReverseAngle());

		driveMotor.setInverted(cfg.isReverseDrive());
		turningMotor.setInverted(cfg.isReverseSteer());
		turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
		turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
		turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
		turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
		turningMotor.burnFlash();
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(getVelocity(), new Rotation2d(turningEncoder.get()));
	}

	public void resetEncoder() {
		driveEncoder.setPosition(0);
	}

	public double getPositionX() {
		return driveEncoder.getPosition() * kWheelRadius * 0.10472 * 0.12 * Math.sin(turningEncoder.get());
	}
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			driveEncoder.getPosition(), new Rotation2d(turningEncoder.get()));
	  }
	public double getPositionY() {
		return driveEncoder.getPosition() * kWheelRadius * 0.10472 * 0.12 * Math.cos(turningEncoder.get());
	}

	public double getVelocity() {
		//return driveEncoder.getVelocity();
		
		return ((-driveEncoder.getVelocity() / 7.73) / 60) * 2 * Math.PI * kWheelRadius;
	}

	public double echo(double val) {
		return val;
	}

	public SwerveModuleState desiredState = null;

	public void setDesiredState(SwerveModuleState state) {
		desiredState = state;

		double dAngle = state.angle.getDegrees() - turningEncoder.getD();
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
		turningMotor.set(turnOutput);
	}

	public void setDesiredStateAuto(SwerveModuleState state) {
		desiredState = state;

		double dAngle = state.angle.getDegrees() - turningEncoder.getD();
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

		// if (cfg.getName() == PivotId.FR) {
		// 	System.out.printf("targetSpeed: %.2f, driveFeedForward: %.2f, driveOutput: %.2f", targetSpeed, driveFeedForward, driveOutput);
			
		// }

		driveMotor.setVoltage(driveFeedForward + driveOutput);
		turningMotor.set(turnOutput);
	}
	public void setAngleD(double angle) {
		double dAngle = angle - turningEncoder.getD();
		double dAngleAbs = Math.abs(dAngle) % 360;
		boolean flipDrive = (90.0 <= dAngleAbs) && (dAngleAbs <= 270.0);
		double sin = Math.sin(dAngle * Math.PI / 180.0);
		sin = (flipDrive) ? -sin : sin;
		double turnOutput = turningPIDController.calculate(sin, 0);

		turningMotor.set(turnOutput);
	}

	public void setSpeed(double speed) {
		driveMotor.set(speed);
	}
}
