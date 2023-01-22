package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utilities.Logger;

import javax.xml.crypto.dsig.keyinfo.X509IssuerSerial;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;

public class Drive extends CommandBase {

    DriveSubsystem driveSubsystem;
    XboxController driverController = new XboxController(0);
    Joystick joystick = new Joystick(0);

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    private double xSpeed;
    private double ySpeed;
    private double rot;

    private boolean fieldRelative = true;

    public Drive(DriveSubsystem driveSubsystem, boolean fieldRelative) {
        this.fieldRelative = fieldRelative;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        if(driverController.getBackButtonPressed()) {
            fieldRelative = !fieldRelative;
            System.out.println("Toggled FC/RC");
        }

        // System.out.println(driveSubsystem.getGyroAngle());
        Trigger leftTrigger = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1);

        if(leftTrigger.getAsBoolean()) {
            xSpeed = -driverController.getLeftY() * 0.55;
            ySpeed = -driverController.getLeftX() * 0.55;
            rot = -driverController.getRightX() * 0.35;
            // System.out.println("oiwehfiowehf");
        } else {
            xSpeed = -m_xspeedLimiter.calculate(driverController.getLeftY());
            ySpeed = -m_yspeedLimiter.calculate(driverController.getLeftX());
            rot = driverController.getRightX();
            // xSpeed = driverController.getLeftY();
            // ySpeed = driverController.getLeftX();
            double linearSpeed = Math.sqrt((xSpeed * xSpeed) + (ySpeed * ySpeed));
            // double alpha = 1 / (Math.min(Math.abs(rot/linearSpeed), Math.abs(linearSpeed/rot)) + 1);
            // xSpeed *= alpha;
            // ySpeed *= alpha;
            // rot *= alpha;
            // System.out.println("xSpeed: " + xSpeed + "\nySpeed: " + ySpeed + "\nrot: " + rot + "\nalpha: " + alpha);
            System.out.println("Gyro: " + driveSubsystem.getGyroAngle().getDegrees());
            rot = -m_rotLimiter.calculate(driverController.getRightX());
            
        }

        if (Math.hypot(xSpeed, ySpeed) < 0.15) {
            xSpeed = 0;
            ySpeed = 0;
        }

        if (Math.abs(rot) < 0.15) {
            rot = 0;
        }

        driveSubsystem.drive(xSpeed, ySpeed, rot * 2, fieldRelative);
    }

    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
