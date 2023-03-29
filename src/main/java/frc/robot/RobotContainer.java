// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auton.paths.ChargeStation;
import frc.robot.auton.paths.PlaceOut;
import frc.robot.sensors.Gyro;
import frc.robot.sensors.LED;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.PixyCam;
import frc.robot.sensors.Resolver;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.Preset;
import frc.robot.subsystems.arm.commands.ArmEndEffectorCommand;
import frc.robot.subsystems.arm.commands.ArmManualCommand;
import frc.robot.subsystems.arm.commands.ArmStopCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.JoystickDriveCommand;
import frc.robot.subsystems.drive.commands.ResetGyroCommand;
import frc.robot.subsystems.drive.commands.ResetOdometryCommand;
import frc.robot.subsystems.drive.commands.Stop;
import frc.robot.subsystems.foot.FootSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.utilities.PresetBoard;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  PixyCam pixyCam = new PixyCam(led);
  DashboardInit dashboardInit;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    setupNetworkTables();

    gyro = new Gyro();
    driveSubsystem = new DriveSubsystem(gyro);
    
    armSubsystem = new ArmSubsystem(lowEncoder, upperEncoder);
    armStopCommand  = new ArmStopCommand(armSubsystem);
    dashboardInit = new DashboardInit(gyro, armSubsystem, driveSubsystem, grabberSubsystem, this);
    driveSubsystem.setDefaultCommand(new JoystickDriveCommand(driveSubsystem, true, gyro, driverController, footSubsystem, pixyCam));
    armSubsystem.setDefaultCommand(new ArmEndEffectorCommand(armSubsystem, operatorController));

    setPreset(Preset.Pickup, armSubsystem.createArmProfileCommand(Preset.Pickup));
    //grabberSubsystem.setServoTurned(false);
    grabberSubsystem.setServoAngle(Constants.ServoSmasAngles.CYMBAL_SERVO_UPRIGHT_ANGLE);


    // Configure the trigger bindings
    configureBindings();

    // Prints (DO NOT DELETE, JUST COMMENT OUT THE PRINTS NOT BEING USED)
    new RepeatCommand(new InstantCommand(
      // () -> {} // do nothing
      () -> driveSubsystem.print()
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
      .whileTrue(new InstantCommand(() -> setPreset(Preset.Travel, armSubsystem.create2dEndEffectorProfileCommand(Preset.Travel, 2, 2, 2, 2))));

    new Trigger(() -> operatorController.getRightBumper()||driverController.getRightBumper())
      .onTrue(new InstantCommand(() -> grabberSubsystem.toggleClamped()));

    new Trigger(()-> operatorController.getPOV() == 0).onTrue(new InstantCommand(() -> grabberSubsystem.incramentServoUp()));
    new Trigger(()-> operatorController.getPOV() == 180).onTrue(new InstantCommand(() -> grabberSubsystem.incramentServoDown()));

    
    new Trigger(() -> presetBoard.povIsUpwards())
      .whileTrue(new InstantCommand(() -> armSubsystem.setIsInCubeMode(false)).andThen(new InstantCommand(() -> led.setStateGreen())));//.andThen(new InstantCommand(() -> led.setStateGreen()))

    new Trigger(() -> presetBoard.povIsDownwards())
      .whileTrue(new InstantCommand(() -> armSubsystem.setIsInCubeMode(true)).andThen(new InstantCommand(() -> led.setStateBlue())));//.andThen(new InstantCommand(() -> led.setStateBlue()))

    new Trigger(() -> operatorController.getAButtonPressed())
      .onTrue(new InstantCommand(
        () -> currentArmCommand.schedule()).alongWith(new InstantCommand(() -> setServo())));

    new Trigger(() -> presetBoard.getRawButton(PresetBoard.Button.kLB))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.Substation, armSubsystem.createEndEffectorProfileCommand(Preset.Substation))));
  
    new Trigger(() -> presetBoard.getAxisButton(PresetBoard.Axis.kLTAxis))
      .whileTrue(armStopCommand);

    new Trigger(() -> presetBoard.getRawButton(PresetBoard.Button.kX))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.HighPlacing, armSubsystem.create2dEndEffectorProfileCommand(Preset.HighPlacing, 1.9, 4.3, 0.6, 2)))); //and then move servo to mid?
    
    new Trigger(() -> presetBoard.getRawButton(PresetBoard.Button.kA))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.MidPlacing, armSubsystem.create2dEndEffectorProfileCommand(Preset.MidPlacing, 1.9, 4.3, 0.6, 2))));

    new Trigger(() -> presetBoard.getRawButton(PresetBoard.Button.kY))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.UprightConeGround, armSubsystem.create2dEndEffectorProfileCommand(Preset.UprightConeGround, 4.3, 1.9, 2, 0.6))));
    
    new Trigger(() -> presetBoard.getRawButton(PresetBoard.Button.kB))
      .whileTrue(new InstantCommand(() -> setPreset(Preset.LowPlacing, armSubsystem.createEndEffectorProfileCommand(Preset.LowPlacing))));
    
    new Trigger(() -> presetBoard.getRawButton(PresetBoard.Button.kRB))
    .whileTrue(new InstantCommand(() -> setPreset(Preset.Ground, armSubsystem.create2dEndEffectorProfileCommand(Preset.Ground, 4.3, 1.9, 2, 0.6))));
    
      new Trigger(() -> presetBoard.getAxisButton(PresetBoard.Axis.kRTAxis) || operatorController.getXButton())
      .whileTrue(new InstantCommand(
        () -> setPreset(
          Preset.Pickup,
          new InstantCommand(
            () -> grabberSubsystem.setClamped(false)
            )
              .andThen(armSubsystem.create2dEndEffectorProfileCommand(Preset.Pickup, 2, 2, 2, 2))
          )
        ));

      new Trigger(() -> driverController.getLeftBumper())
      .whileTrue(new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d(0, 0, gyro.getRotation2d()))));

      new Trigger(() -> operatorController.getYButton())
      .whileTrue(new InstantCommand(() -> setPreset(Preset.Travel, armSubsystem.create2dEndEffectorProfileCommand(Preset.Travel, 1.9, 4.3, 0.6, 2.0))));
      // new Trigger(() -> operatorController.getBButton())
      //.onTrue(new InstantCommand(() -> grabberSubsystem.setServoTurned(true)))
      //.onFalse(new InstantCommand(() -> grabberSubsystem.setServoTurned(false)));
  }

  public void firstEnabled(){
    if (wasEnabled){
      return;
    }
    wasEnabled = true;
    // DataLogManager.log("Robot was enabled for the first time.");
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
     if (currentPreset == Preset.Pickup){
      grabberSubsystem.setServoOffset(Constants.ServoSmasAngles.CYMBAL_SERVO_UPRIGHT_ANGLE); //TODO: make this the correct constant (is set to ground)

    }
    else{
      grabberSubsystem.setServoOffset(0); //TODO is right?
    } 
    presetPub.set(currentPreset.toString());
  }

  public void setServo(){
    if (currentPreset == Preset.MidPlacing){
      grabberSubsystem.servoMove(-60); // TODO Constant
      //grabberSubsystem.setServoAngle(Constants.ServoSmasAngles.CYMBAL_SERVO_MID_ANGLE);
    }
    else if (currentPreset == Preset.HighPlacing){
      grabberSubsystem.servoMove(-90); //TODO fix constant
      //grabberSubsystem.setServoAngle(Constants.ServoSmasAngles.CYMBAL_SERVO_HIGH_ANGLE);
    }

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