package frc.robot.auton;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.Preset;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.ChangeGrabState;
import frc.robot.subsystems.grabber.commands.SetGrabCommand;
import frc.robot.subsystems.grabber.commands.UnGrab;

public class CreateEventMap {
    public static HashMap<String, Command> EventMap(GrabberSubsystem grabberSubsystem, ArmSubsystem armSubsystem, DriveSubsystem swerve){
        HashMap<String, Command> eventMap = new HashMap<>();
        Command place =  armSubsystem.create2dEndEffectorProfileCommandNoInstant(
            Preset.HighPlacing,1.9, 4.3, 0.6, 2);
        eventMap.put("PlaceHigh", place.alongWith(new SequentialCommandGroup(new WaitCommand(
            Constants.ServoSmasAngles.SERVO_WAIT),new InstantCommand(
            () -> grabberSubsystem.servoMove(Constants.ServoSmasAngles.HIGH_ANGLE)))));

        eventMap.put("Ungrab", new UnGrab(grabberSubsystem));
        eventMap.put("Wait0.4", new WaitCommand(0.4));
        eventMap.put("Grab", new ChangeGrabState(grabberSubsystem, true));
        eventMap.put("Wait0.2", new WaitCommand(0.2));
        eventMap.put("StopDriving", new InstantCommand(() -> swerve.drive(0, 0, 0, false)));
        eventMap.put("UprightCone", armSubsystem.createEndEffectorProfileCommandNoInstant(Preset.UprightConeGround));
        eventMap.put("LowPlacing", armSubsystem.createEndEffectorProfileCommandNoInstant(Preset.LowPlacing));
        eventMap.put("Wait1", new WaitCommand(1));
        return eventMap;
    }
}
