package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.JoystickCleaner;
import frc.robot.subsystems.foot.FootSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class JoystickDriveCommand extends CommandBase {
    final double SLOW_LINEAR_SPEED = 0.7;
    final double SLOW_ROTATIONAL_SPEED = 0.55;

    final double LOWER_DB = 0.15;
    final double UPPER_DB = 0.15;

    DriveSubsystem driveSubsystem;
    Gyro gyro;
    XboxController driverController;

    FootSubsystem footSubsystem;

    JoystickCleaner joystickCleaner = new JoystickCleaner();

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    private boolean fieldRelative = true;

    public JoystickDriveCommand(DriveSubsystem driveSubsystem, boolean fieldRelative, Gyro gyro, XboxController driveController, FootSubsystem footSubsystem) {
        this.fieldRelative = fieldRelative;
        this.driveSubsystem = driveSubsystem;
        this.gyro = gyro;
        this.driverController = driveController;
        this.footSubsystem = footSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double xSpeed;
        double ySpeed;
        double rot;
        boolean isLimitSwitch = false;
        isLimitSwitch = footSubsystem.getLimitSwitch();

        

        if (driverController.getBackButtonPressed()) {
            fieldRelative = !fieldRelative;
        }

        Trigger leftTrigger = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1);

        if(leftTrigger.getAsBoolean()) {
            xSpeed = -driverController.getLeftY() * SLOW_LINEAR_SPEED;
            ySpeed = -driverController.getLeftX() * SLOW_LINEAR_SPEED;
            rot = -driverController.getRightX() * SLOW_ROTATIONAL_SPEED;
        } else {
            xSpeed = -m_xspeedLimiter.calculate(driverController.getLeftY());
            ySpeed = -m_yspeedLimiter.calculate(driverController.getLeftX());
            rot = -m_rotLimiter.calculate(driverController.getRightX());
        }

        /* Apply linear deadband */
        joystickCleaner.setX(xSpeed);
        joystickCleaner.setY(ySpeed);
        joystickCleaner.applyDeadband(LOWER_DB, UPPER_DB);
        xSpeed = joystickCleaner.getX();
        ySpeed = joystickCleaner.getY();

        /* Apply rotational deadband */
        rot = MathUtil.applyDeadband(rot, LOWER_DB);

        /* Increase rotational sensitivity */
        rot = Math.signum(rot) * Math.pow(Math.abs(rot), 1.0 / 3.0);
        // if (xSpeed != 0 || ySpeed != 0){
        //     if (Math.abs(xSpeed) >= Math.abs(ySpeed)){
        //         xSpeed = Math.signum(xSpeed) * 0.5;
        //         ySpeed = 0;
        //     }
        //     else{
        //         ySpeed = Math.signum(ySpeed) * 0.5;
        //         xSpeed = 0;
        //     }
        // }

        if (!isLimitSwitch) {
            driveSubsystem.drive(xSpeed, ySpeed, rot, fieldRelative);
        } 
        else {
            driveSubsystem.drive(0, 0, 0, fieldRelative);
        }
        
    }
    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
