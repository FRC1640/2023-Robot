// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auton.paths.ChargeStation;
import frc.robot.auton.paths.PlaceOut;
import frc.robot.sensors.Gyro;
import frc.robot.sensors.LED;
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
import frc.robot.subsystems.drive.commands.Stop;
import frc.robot.subsystems.foot.FootSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.utilities.PresetBoard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  Gyro gyro;
  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1);
  PresetBoard presetBoard = new PresetBoard(2);
  boolean wasEnabled = false;
  DriveSubsystem driveSubsystem;
  Limelight limelight = new Limelight();
  ArmSubsystem armSubsystem;
  Compressor pcmCompressor= new Compressor(0, PneumaticsModuleType.CTREPCM);
  GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
  FootSubsystem footSubsystem = new FootSubsystem();

  Resolver lowEncoder = new Resolver(4, 0.25, 4.75, -180, false);
  Resolver upperEncoder = new Resolver(5, 0.25, 4.75, -180, true);
  Preset currentPreset;
  Command currentArmCommand;

  Command armStopCommand;
  LED led = new LED();
  DashboardInit dashboardInit;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    setupNetworkTables();

    gyro = new Gyro();
    driveSubsystem = new DriveSubsystem(gyro);
    
    armSubsystem = new ArmSubsystem(lowEncoder, upperEncoder);
    armStopCommand  = new ArmStopCommand(armSubsystem);
    dashboardInit = new DashboardInit(gyro, armSubsystem, driveSubsystem, grabberSubsystem, this);
    driveSubsystem.setDefaultCommand(new JoystickDriveCommand(driveSubsystem, true, gyro, driverController));
    armSubsystem.setDefaultCommand(new ArmEndEffectorCommand(armSubsystem, operatorController));

    setPreset(Preset.Pickup, armSubsystem.createArmProfileCommand(Preset.Pickup));

    // Configure the trigger bindings
    configureBindings();

    // Prints (DO NOT DELETE, JUST COMMENT OUT THE PRINTS NOT BEING USED)
    new RepeatCommand(new InstantCommand(
      () -> {} // do nothing
      // () -> System.out.println(currentPreset)
      // () -> System.out.println("POV: " + presetBoard.getPOV())
      // () -> System.out.format("%s, %.2f, %.2f\n", armSubsystem.getEndEffectorPosition().toString(), armSubsystem.getLowerPosition(), armSubsystem.getUpperPosition())
    )).ignoringDisable(true).schedule();
  }

  private void configureBindings() {

    new Trigger(() -> driverController.getYButton())
    .whileTrue(new Stop(driveSubsystem));

    new Trigger(() -> driverController.getXButton())
      .onTrue(new InstantCommand(() -> footSubsystem.toggleClamped()));

    new Trigger(() -> driverController.getStartButtonPressed())
      .onTrue(new ResetGyroCommand(gyro));

    new Trigger(() -> (operatorController.getLeftTriggerAxis() > 0.7))
      .whileTrue(new ArmManualCommand(armSubsystem, operatorController));

    new Trigger(() -> operatorController.getBButton())
      .whileTrue(armStopCommand);

    new Trigger(() -> operatorController.getRightBumper())
      .onTrue(new InstantCommand(() -> grabberSubsystem.toggleClamped()));
    
    new Trigger(() -> presetBoard.povIsUpwards())
      .whileTrue(new InstantCommand(() -> armSubsystem.setIsInCubeMode(false)).andThen(new InstantCommand(() -> led.setStateGreen())));//.andThen(new InstantCommand(() -> led.setStateGreen()))

    new Trigger(() -> presetBoard.povIsDownwards())
      .whileTrue(new InstantCommand(() -> armSubsystem.setIsInCubeMode(true)).andThen(new InstantCommand(() -> led.setStateBlue())));//.andThen(new InstantCommand(() -> led.setStateBlue()))

    new Trigger(() -> operatorController.getAButtonPressed())
      .onTrue(new InstantCommand(
        () -> currentArmCommand.schedule()));

    // new Trigger(() -> presetBoard.getRawButton(PresetBoard.Button.kLB))
    //   .whileTrue(new InstantCommand(() -> setPreset(Preset.Substation, armSubsystem.createEndEffectorProfileCommand(Preset.Substation))));
    
    new Trigger(() -> presetBoard.getAxisButton(PresetBoard.Axis.kLTAxis))
      .whileTrue(armStopCommand);

    new Trigger(() -> presetBoard.getRawButton(PresetBoard.Button.kX))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.HighPlacing, armSubsystem.createEndEffectorProfileCommand(Preset.HighPlacing))));
    
    new Trigger(() -> presetBoard.getRawButton(PresetBoard.Button.kA))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.MidPlacing, armSubsystem.createEndEffectorProfileCommand(Preset.MidPlacing))));

    new Trigger(() -> presetBoard.getRawButton(PresetBoard.Button.kY))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.UprightConeGround, armSubsystem.createEndEffectorProfileCommand(Preset.UprightConeGround))));
    
    new Trigger(() -> presetBoard.getRawButton(PresetBoard.Button.kB))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.LowPlacing, armSubsystem.createEndEffectorProfileCommand(Preset.LowPlacing))));
    
    new Trigger(() -> presetBoard.getRawButton(PresetBoard.Button.kRB))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.Ground, armSubsystem.createEndEffectorProfileCommand(Preset.Ground))));
    
      new Trigger(() -> presetBoard.getAxisButton(PresetBoard.Axis.kRTAxis) || operatorController.getXButton())
      .whileTrue(new InstantCommand(
        () -> setPreset(
          Preset.Pickup,
          new InstantCommand(
            () -> grabberSubsystem.setClamped(false)
          )
            .andThen(armSubsystem.createEndEffectorProfileCommand(Preset.Pickup))
        )
      ));

      new Trigger(() -> operatorController.getYButton())
      .whileTrue(new InstantCommand(
        () -> setPreset(
          Preset.Travel,
          new InstantCommand(
            () -> grabberSubsystem.setClamped(false)
          )
            .andThen(armSubsystem.createEndEffectorProfileCommand(Preset.Travel))
        )
      ));
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
    Command autoCommand = dashboardInit.getAuton();

    // Run path following command, then stop at the end.
    return autoCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, false));
  }

  public void setPreset(Preset preset, Command armCommand){
    currentPreset = preset;
    currentArmCommand = armCommand;
    presetPub.set(currentPreset.toString());
  }

  public Preset getCurrentPreset(){
    return currentPreset;
  }

  NetworkTableInstance nt;
  NetworkTable table;
  StringPublisher presetPub;

  private void setupNetworkTables() {
    nt = NetworkTableInstance.getDefault();
    table = nt.getTable("robot");
    presetPub = table.getStringTopic("currentPreset").publish();
  }
}