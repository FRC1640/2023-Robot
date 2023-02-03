package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.subsystems.drive.JoystickCleaner;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.controller.PIDController;


public class JoystickDriveCommand extends CommandBase {
    final double SLOW_LINEAR_SPEED = 0.55;
    final double SLOW_ROTATIONAL_SPEED = 0.55;

    final double LOWER_DB = 0.15;
    final double UPPER_DB = 0.15;

    DriveSubsystem driveSubsystem;
    Gyro gyro;
    XboxController driverController;
    
    Trigger leftTrigger = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1);

    JoystickCleaner joystickCleaner = new JoystickCleaner();

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private boolean fieldRelative = true;

    private boolean gyroCorrectionOn = false;
    private double gyroCorrectionHeading;
    private PIDController gyroCorrectionPID = new PIDController(0.5, 0, 0);

    public JoystickDriveCommand(DriveSubsystem driveSubsystem, boolean fieldRelative, Gyro gyro, XboxController driveController) {
        this.fieldRelative = fieldRelative;
        this.driveSubsystem = driveSubsystem;
        this.gyro = gyro;
        this.driverController = driveController;
        addRequirements(driveSubsystem);

        gyroCorrectionPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        gyroCorrectionOn = false;
    }

    @Override
    public void execute() {
        double xSpeed;
        double ySpeed;
        double rot;

        if (driverController.getBackButtonPressed()) {
            fieldRelative = !fieldRelative;
            System.out.format("%b\n", fieldRelative);
        }

        if(leftTrigger.getAsBoolean()) {
            xSpeed = -driverController.getLeftY() * SLOW_LINEAR_SPEED;
            ySpeed = -driverController.getLeftX() * SLOW_LINEAR_SPEED;
            rot = -driverController.getRightX() * SLOW_ROTATIONAL_SPEED;
        } else {
            xSpeed = -driverController.getLeftY();
            ySpeed = -driverController.getLeftX();
            rot = -driverController.getRightX();
        }
        // System.out.format("%.2f\n", Math.toDegrees(Math.atan2(ySpeed, xSpeed)));

        /* Apply linear deadband */
        joystickCleaner.setX(xSpeed);
        joystickCleaner.setY(ySpeed);
        joystickCleaner.applyDeadband(LOWER_DB, UPPER_DB);
        xSpeed = joystickCleaner.getX();
        ySpeed = joystickCleaner.getY();

        /* Apply rotational deadband */
        rot = MathUtil.applyDeadband(rot, LOWER_DB, 1 - UPPER_DB);

        /* Increase rotational sensitivity */
        rot = Math.signum(rot) * Math.pow(Math.abs(rot), 1.0 / 3.0);

        System.out.format("%.2f\n", Math.toDegrees(Math.atan2(ySpeed, xSpeed)));

        /* Determine whether to apply gyro correction */
        if(Math.abs(rot) < 0.01 && Math.abs(gyro.getGyroRoll()) < 5 && Math.abs(gyro.getGyroPitch()) < 5){
            if (!gyroCorrectionOn) {
                gyroCorrectionOn = true;
                gyroCorrectionHeading = gyro.getGyroAngle().getRadians();
            }
        }
        else{
            gyroCorrectionOn = false;
        }

        /* Apply gyro correction */
        if (gyroCorrectionOn) {
            // rot = gyroCorrectionPID.calculate(gyro.getGyroAngle().getRadians(), gyroCorrectionHeading);
        }

        // System.out.format("%.2f, %.2f, %s\n", Math.toDegrees(gyroCorrectionHeading), rot, Boolean.toString(gyroCorrectionOn));
        // System.out.format("%.2f, %.2f\n", Math.hypot(xSpeed, ySpeed), rot);
        // System.out.format("%.2f\n", Math.toDegrees(Math.atan2(ySpeed, xSpeed)));

        driveSubsystem.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
