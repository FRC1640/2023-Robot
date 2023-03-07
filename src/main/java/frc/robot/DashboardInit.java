package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auton.paths.Place;
import frc.robot.auton.paths.PlaceCharge;
import frc.robot.auton.paths.PlaceOut;
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
        sChooser.addOption("Place out", new PlaceOut().loadAuto(gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        sChooser.addOption("Place charge", new PlaceCharge().loadAuto(gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        sChooser.addOption("Place", new Place().loadAuto(gyro, driveSubsystem, armSubsystem, grabberSubsystem));
        // sChooser.addOption("Null", null);
        autonTab.add(sChooser).withSize(5, 5).withPosition(0, 0);
        driveTab.addString("Preset", () -> robotContainer.getCurrentPreset().name());
        driveTab.addBoolean("IsInCubeMode", () -> armSubsystem.getCubeMode()); //use .withProperties for color

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