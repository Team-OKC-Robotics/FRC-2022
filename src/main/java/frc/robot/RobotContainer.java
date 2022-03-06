// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.DoNothingAuto;
import frc.robot.autos.DriveOffLineAuto;
import frc.robot.autos.GyroTestAuto;
import frc.robot.commands.drivetrain.TeleOpDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutoChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 * 
 * @param <setClimberCommand>
 * @param <SetClimbCommand>
 */
public class RobotContainer<setClimberCommand, SetClimbCommand> {
  // gamepads
  private final Joystick gamepad1 = new Joystick(0);
  private final Joystick gamepad2 = new Joystick(1);

  // buttons
  private JoystickButton aButton = new JoystickButton(gamepad1, 1);
  private JoystickButton bButton = new JoystickButton(gamepad1, 2);
  private JoystickButton xButton = new JoystickButton(gamepad1, 3);
  private JoystickButton yButton = new JoystickButton(gamepad1, 4);
  private JoystickButton leftBumper = new JoystickButton(gamepad1, 5);
  private JoystickButton rightBumper = new JoystickButton(gamepad1, 6);
  private JoystickButton backButton = new JoystickButton(gamepad1, 7);
  private JoystickButton startButton = new JoystickButton(gamepad1, 8);
  private JoystickButton rightStickButton = new JoystickButton(gamepad1, 10);
  private JoystickButton leftStickButton = new JoystickButton(gamepad1, 9);

  private JoystickButton aButton2 = new JoystickButton(gamepad2, 1);
  private JoystickButton bButton2 = new JoystickButton(gamepad2, 2);
  private JoystickButton xButton2 = new JoystickButton(gamepad2, 3);
  private JoystickButton yButton2 = new JoystickButton(gamepad2, 4);
  private JoystickButton leftBumper2 = new JoystickButton(gamepad2, 5);
  private JoystickButton rightBumper2 = new JoystickButton(gamepad2, 6);
  private JoystickButton backButton2 = new JoystickButton(gamepad2, 7);
  private JoystickButton startButton2 = new JoystickButton(gamepad2, 8);
  private JoystickButton rightStickButton2 = new JoystickButton(gamepad2, 10);
  private JoystickButton leftStickButton2 = new JoystickButton(gamepad2, 9);

  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  // autos
  private final DoNothingAuto doNothingAuto = new DoNothingAuto(drivetrain); // drives the robot 0 inches
  private final DriveOffLineAuto driveOffLine = new DriveOffLineAuto(drivetrain); // drives the robot forwards
  private final GyroTestAuto gyroTestAuto = new GyroTestAuto(drivetrain);

  // commands
  private final TeleOpDriveCommand teleOpDrive = new TeleOpDriveCommand(drivetrain, gamepad1);
  
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // add the autos to the chooser
    AutoChooser.addAutos(doNothingAuto, driveOffLine, /* shootThenDrive, twoBallAuto, threeBallAuto, */ gyroTestAuto);
    AutoChooser.addGamepad(gamepad1);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      // drivetrain commands
      backButton.whenPressed(teleOpDrive);

      // probably don't need these
      // aButton.whenPressed(ledsOn);
      // bButton.whenPressed(ledsOff);
  }

  public Command getDriveCommand() {
    // return teleopDrive;
    return teleOpDrive;
  }
}
