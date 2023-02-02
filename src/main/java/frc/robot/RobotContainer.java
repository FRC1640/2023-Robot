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
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.Drive;
import frc.robot.subsystems.drive.commands.ResetGyro;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  Gyro gyro;
  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1);
  Joystick driveJoystick = new Joystick(0);
  Joystick opJoystick = new Joystick(1);
  boolean wasEnabled = false;
  DriveSubsystem drive;
  Limelight limelight = new Limelight();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    gyro = new Gyro();
    drive = new DriveSubsystem(gyro, limelight);
    // Configure the trigger bindings
    drive.setDefaultCommand(new Drive(drive, true, gyro, driverController, driveJoystick));
    configureBindings();
  }

  private void configureBindings() {
    JoystickButton startButton = new JoystickButton(driveJoystick, 8);
    startButton.onTrue(new ResetGyro(drive, gyro));
  }
  public void firstEnabled(){
    if (wasEnabled){
      return;
    }
    gyro.resetGyro();
    wasEnabled = true;
    DataLogManager.log("first enabled method ran");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //An example command will be run in autonomous
    AlignAuto auto = new AlignAuto();
    return auto.loadAuto(gyro, drive, limelight);
  }
}
