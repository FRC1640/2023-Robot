// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.auton.commands.Align;
import frc.robot.auton.paths.AlignAuto;
import frc.robot.auton.paths.ChargeStation;
import frc.robot.sensors.Gyro;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Resolver;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.Preset;
import frc.robot.subsystems.arm.commands.ArmEndEffectorCommand;
import frc.robot.subsystems.arm.commands.ArmManualCommand;
import frc.robot.subsystems.arm.commands.ArmStopCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.JoystickDriveCommand;
import frc.robot.subsystems.drive.commands.ResetGyroCommand;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.commands.TeleopGrabberCommand;

import java.time.Instant;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  Gyro gyro;
  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1);
  GenericHID presetBoard = new GenericHID(2);
  boolean wasEnabled = false;
  DriveSubsystem driveSubsystem;
  Limelight limelight = new Limelight();
  ArmSubsystem armSubsystem;
  Compressor pcmCompressor= new Compressor(0, PneumaticsModuleType.CTREPCM);
  GrabberSubsystem grabberSubsystem = new GrabberSubsystem();

  Resolver lowEncoder = new Resolver(4, 0.25, 4.75, -180, false);
  Resolver upperEncoder = new Resolver(5, 0.25, 4.75, -180, true);
  boolean cubeMode;
  Preset currentPreset = Preset.Pickup;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    gyro = new Gyro();
    driveSubsystem = new DriveSubsystem(gyro);
    
    armSubsystem = new ArmSubsystem(lowEncoder, upperEncoder);
    DashboardInit dashboardInit = new DashboardInit(gyro, armSubsystem);
    driveSubsystem.setDefaultCommand(new JoystickDriveCommand(driveSubsystem, true, gyro, driverController));
    armSubsystem.setDefaultCommand(new ArmStopCommand(armSubsystem));


    new RunCommand(() -> driveSubsystem.print())//
     .ignoringDisable(true).cancel();
    
    // drive.setDefaultCommand(new SetDriveDirect(drive, driverController));
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    JoystickButton startButton = new JoystickButton(driverController, 8);
    startButton.onTrue(new ResetGyroCommand(gyro));

    Trigger manualCommandTrigger = new Trigger(() -> (operatorController.getLeftTriggerAxis() > 0.7));
    manualCommandTrigger.whileTrue(new ArmManualCommand(armSubsystem, operatorController));

    Trigger endEffectorCommandTrigger = new Trigger(() -> (operatorController.getRightTriggerAxis() > 0.7));
    endEffectorCommandTrigger.and(manualCommandTrigger.negate())
      .whileTrue(new ArmEndEffectorCommand(armSubsystem, operatorController));

    new Trigger(() -> operatorController.getRightBumper())
      .onTrue(new InstantCommand(
        () -> grabberSubsystem.toggleClamped()
      ));
    
    new Trigger(() -> ((presetBoard.getPOV() <= 45 || presetBoard.getPOV() >= 315) && presetBoard.getPOV() >= 0) || operatorController.getYButtonPressed())
      .whileTrue(new InstantCommand(() -> armSubsystem.setIsInCubeMode(false)));

    new Trigger(() -> ((presetBoard.getPOV() >= 135 && presetBoard.getPOV() <= 225)&& presetBoard.getPOV() >= 0) || operatorController.getXButtonPressed())
      .whileTrue(new InstantCommand(() -> armSubsystem.setIsInCubeMode(true)));

    new Trigger(() -> operatorController.getAButtonPressed())
      .onTrue(new InstantCommand(
        () -> armSubsystem.createArmProfileCommand(currentPreset).schedule()
      ));

    new Trigger(() -> presetBoard.getRawButtonPressed(5))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.Substation)));
    
    // new Trigger(() -> presetBoard.getRawAxis(2) == 1)
    //   .whileTrue(new InstantCommand(() -> setPreset(NONE))); RESERVED

    new Trigger(() -> presetBoard.getRawButtonPressed(3))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.HighPlacing)));
    
    new Trigger(() -> presetBoard.getRawButtonPressed(1))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.MidPlacing)));

    new Trigger(() -> presetBoard.getRawButtonPressed(4))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.UprightConeGround)));
    
    new Trigger(() -> presetBoard.getRawButtonPressed(2))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.LowPlacing)));
    
    new Trigger(() -> presetBoard.getRawButtonPressed(6))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.Ground)));
    
      new Trigger(() -> presetBoard.getRawAxis(3) == 1)
      .whileTrue(new InstantCommand(() -> setPreset(Preset.Pickup)));


    // new Trigger(() -> presetBoard.getRawButtonPressed(1))
    //   .toggleOnTrue(armSubsystem.createArmProfileCommand(Preset.Ground));

    // new Trigger(() -> presetBoard.getRawButtonPressed(2))
    //   .toggleOnTrue(armSubsystem.createArmProfileCommand(Preset.Travel));

    // new Trigger(() -> presetBoard.getRawButtonPressed(3))
    //   .toggleOnTrue(armSubsystem.createArmProfileCommand(Preset.Pickup));

    // Trigger switchToCube = new Trigger(() -> presetBoard.getRawAxis(2) == 1);
    // switchToCube.whileTrue(new RunCommand(() -> setMode(true)));

    // Trigger switchToCone = new Trigger(() -> presetBoard.getRawButtonPressedPressed(5));
    // switchToCone.whileTrue(new RunCommand(() -> setMode(false)));


    new RepeatCommand(new InstantCommand(
      () -> System.out.println(currentPreset)
      // () -> System.out.println("POV: " + presetBoard.getPOV())
      // () -> System.out.format("%s, %.2f, %.2f\n", armSubsystem.getEndEffectorPosition().toString(), armSubsystem.getLowerPosition(), armSubsystem.getUpperPosition())
    )).ignoringDisable(true).schedule();
  }

  public void firstEnabled(){
    if (wasEnabled){
      return;
    }
    gyro.resetGyro();
    wasEnabled = true;
    DataLogManager.log("Robot was enabled for the first time.");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //An example command will be run in autonomous
    ChargeStation auto = new ChargeStation();
    return auto.loadAuto(gyro, driveSubsystem);
  }
  public void setMode(boolean m){
    cubeMode = m;
    System.out.println(cubeMode);
  }
  public void setPreset(Preset preset){
    currentPreset = preset;
  }
  public Preset getCurrentPreset(){
    return currentPreset;
  }
}