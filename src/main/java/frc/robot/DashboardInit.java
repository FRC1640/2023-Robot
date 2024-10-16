package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auton.paths.Bump;
import frc.robot.auton.paths.ChargePickup;
import frc.robot.auton.paths.NoBump;
import frc.robot.auton.paths.NoBumpTriple;
import frc.robot.auton.paths.OldPaths.DriveTest;
import frc.robot.auton.paths.OldPaths.Place;
import frc.robot.auton.paths.OldPaths.PlaceCharge;
import frc.robot.auton.paths.OldPaths.PlaceCharge2;
import frc.robot.auton.paths.OldPaths.PlaceOut;
import frc.robot.auton.paths.OldPaths.PlaceOutPickupLeft;
import frc.robot.auton.paths.OldPaths.PlaceOutPickupRight;
import frc.robot.auton.paths.OldPaths.RedPlaceCharge2;
import frc.robot.auton.paths.OldPaths.RedPlaceOutPickupLeft;
import frc.robot.auton.paths.OldPaths.RedPlaceOutPickupRight;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class DashboardInit {
    private SendableChooser<Command> sChooser;
    public DashboardInit(Gyro gyro, ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem, GrabberSubsystem grabberSubsystem, RobotContainer robotContainer){
        ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
        ShuffleboardTab autonTab = Shuffleboard.getTab("Auton");
        sChooser = new SendableChooser<Command>();
        // sChooser.addOption("Place out", new PlaceOut().loadAuto(gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        sChooser.addOption("Charge", new PlaceCharge().loadAuto(gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        // sChooser.addOption("Charge Pickup", new PlaceCharge2().loadAuto(gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        // sChooser.addOption("Place only", new Place().loadAuto(gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        // sChooser.addOption("2 Place Left", new PlaceOutPickupLeft().loadAuto(gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        // sChooser.addOption("2 Place Right", new PlaceOutPickupRight().loadAuto( gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        sChooser.addOption("ChargePickup", new ChargePickup().loadAuto(gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        sChooser.addOption("RED 2 Place Right", new RedPlaceOutPickupRight().loadAuto( gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        sChooser.addOption("RED 2 Place Left", new RedPlaceOutPickupLeft().loadAuto(gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        // sChooser.addOption("RED Charge Pickup", new RedPlaceCharge2().loadAuto(gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        sChooser.addOption("No Bump", new NoBump().loadAuto(gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        sChooser.addOption("No Bump Triple", new NoBumpTriple().loadAuto(gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        sChooser.addOption("Bump", new Bump().loadAuto(gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        // sChooser.addOption("Null", null);
        autonTab.add(sChooser).withSize(5, 5).withPosition(0, 0);
        driveTab.addString("Preset", () -> robotContainer.getCurrentPreset().name());
        driveTab.addBoolean("IsInCubeMode", () -> armSubsystem.getCubeMode()); //use .withProperties for color
        CameraServer.startAutomaticCapture();
        // driveTab.addCamera("Drive Camera", "USB Camera", "usb:/dev/video0").withSize(3, 3);

        /*
         * things we need:
         * Camera(s) (idk if on robot)
         * Cone or cube mode (needs testing + color + position)
         * Selected preset (needs testing + position)
         * Auto selection (most important) (needs testing)
         */
        // testTab.addDouble("Arm position", () -> System.currentTimeMillis()).withWidget(BuiltInWidgets.kGraph).withSize(5, 5).withPosition(0, 0);
        // testTab.add("Gyro", gyro.getGyro()).withSize(5, 5).withPosition(0, 0).withWidget(BuiltInWidgets.kGyro);
    }
    public Command getAuton(){
        return sChooser.getSelected();
    }
}