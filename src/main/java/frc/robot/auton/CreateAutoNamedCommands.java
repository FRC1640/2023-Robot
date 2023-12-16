package frc.robot.auton;

import java.util.HashMap;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.Preset;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.ChangeGrabState;
import frc.robot.subsystems.grabber.commands.GrabberAutomatic;
import frc.robot.subsystems.grabber.commands.GrabberSpin;
import frc.robot.subsystems.grabber.commands.SetGrabCommand;
import frc.robot.subsystems.grabber.commands.UnGrab;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.commands.RunWristToPosition;

public class CreateAutoNamedCommands {
    public static void EventMap(GrabberSubsystem grabberSubsystem, ArmSubsystem armSubsystem, DriveSubsystem swerve, WristSubsystem wristSubsystem){
        Command place =  armSubsystem.create2dEndEffectorProfileCommandNoInstant(
            Preset.HighPlacing,1.9, 4.3, 0.6, 2);

        NamedCommands.registerCommand("PlaceHigh", place.alongWith(new SequentialCommandGroup(new WaitCommand(
            Constants.ServoSmasAngles.SERVO_WAIT),new InstantCommand(
            () -> grabberSubsystem.servoMove(Constants.ServoSmasAngles.HIGH_ANGLE)))));
        NamedCommands.registerCommand("Ungrab", new UnGrab(grabberSubsystem));
        NamedCommands.registerCommand("Wait0.4", new WaitCommand(0.4));
        NamedCommands.registerCommand("Grab", new ChangeGrabState(grabberSubsystem, true));
        NamedCommands.registerCommand("Wait0.2", new WaitCommand(0.2));
        NamedCommands.registerCommand("Wait0.2", new WaitCommand(0.2));
        NamedCommands.registerCommand("StopDriving", new InstantCommand(() -> swerve.drive(0, 0, 0, false)));
        NamedCommands.registerCommand("UprightCone", armSubsystem.createEndEffectorProfileCommandNoInstant(Preset.UprightConeGround));
        NamedCommands.registerCommand("LowPlacing", armSubsystem.createEndEffectorProfileCommandNoInstant(Preset.LowPlacing));
        NamedCommands.registerCommand("Wait1", new WaitCommand(1));
        WaitCommand w = new WaitCommand(0.5);
        NamedCommands.registerCommand("RollerPlace", new ParallelDeadlineGroup(w, w, new GrabberSpin(grabberSubsystem, 0.5)));
        WaitCommand w2 = new WaitCommand(90);
        NamedCommands.registerCommand("Intake", new ParallelDeadlineGroup(w2, w2, new GrabberAutomatic(grabberSubsystem, new XboxController(3))));;
        Command highPlace = armSubsystem.create2dEndEffectorProfileCommandNoInstant(Preset.HighPlacing, 1.9, 4.3, 0.6, 2); //and then move servo to mid?
        WaitCommand w1 = new WaitCommand(2);
        NamedCommands.registerCommand("RollerHigh", highPlace.alongWith(new ParallelCommandGroup(new ParallelDeadlineGroup(w1,w1, new GrabberSpin(grabberSubsystem, -0.2)),
            new InstantCommand(() -> new RunWristToPosition(wristSubsystem, armSubsystem.getPresetWrist(Preset.HighPlacing)).schedule()))));
        
        
        Command groundPickup = armSubsystem.createEndEffectorProfileCommandNoInstant(Preset.Ground); //and then move servo to mid?
        NamedCommands.registerCommand("RollerGround", groundPickup.alongWith(new InstantCommand(() -> new RunWristToPosition(wristSubsystem, armSubsystem.getPresetWrist(Preset.Ground)).schedule())));



        Command pickupTravel = armSubsystem.createEndEffectorProfileCommandNoInstant(Preset.Pickup); //and then move servo to mid?
        NamedCommands.registerCommand("RollerPickupTravel", pickupTravel.alongWith(new InstantCommand(() -> new RunWristToPosition(wristSubsystem, armSubsystem.getPresetWrist(Preset.Pickup)).schedule())));        
        Command travel = armSubsystem.createEndEffectorProfileCommandNoInstant(Preset.Travel); //and then move servo to mid?
        NamedCommands.registerCommand("RollerTravel", travel.alongWith(new InstantCommand(() -> new RunWristToPosition(wristSubsystem, armSubsystem.getPresetWrist(Preset.Travel)).schedule())));
    }
}
