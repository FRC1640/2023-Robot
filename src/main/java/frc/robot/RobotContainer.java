// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.auton.commands.Align;
import frc.robot.auton.paths.AlignAuto;
import frc.robot.auton.paths.ChargeStation;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.ResetGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1);
  Joystick joystick = new Joystick(0);
  Joystick opJoystick = new Joystick(1);
  public static final DriveSubsystem drive = new DriveSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    drive.initDefaultCommand();
    configureBindings();
  }

  private void configureBindings() {
    JoystickButton startButton = new JoystickButton(joystick, 8);
    startButton.onTrue(new ResetGyro(drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //An example command will be run in autonomous
    ChargeStation auto = new ChargeStation();
    return auto.loadAuto();
  }
}
