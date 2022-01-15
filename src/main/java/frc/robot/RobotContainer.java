// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.autos.*;
import frc.robot.commands.intake.SetIntakeCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.AutoChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // gamepads
  private final Joystick gamepad1 = new Joystick(0);

  // buttons
  private JoystickButton aButton = new JoystickButton(gamepad1, 1);
  private JoystickButton backButton = new JoystickButton(gamepad1, 8);

  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  // autos
  private final DoNothingAuto doNothingAuto = new DoNothingAuto(drivetrain); // drives the robot 0 inches
  private final DriveOffLineAuto driveOffLine = new DriveOffLineAuto(drivetrain); // drives the robot forwards
  private final ShootThenDriveAuto shootThenDrive = new ShootThenDriveAuto(drivetrain, shooter); // drives the robot backwards
  private final TwoBallAuto twoBallAuto = new TwoBallAuto(drivetrain, shooter, intake); // drives the robot backwards
  private final ThreeBallAuto threeBallAuto = new ThreeBallAuto(drivetrain, shooter, intake); // drives the robot backwards
  

  // commands
  //private final RunCommand teleopDrive = new RunCommand(() -> drivetrain.arcadeDrive(-gamepad1.getRawAxis(4), gamepad1.getRawAxis(1)), drivetrain);
  private final RunCommand teleopDrive = new RunCommand(() -> drivetrain.tankDrive(-gamepad1.getRawAxis(1), gamepad1.getRawAxis(5)), drivetrain);

  private final SetIntakeCommand intakeIn = new SetIntakeCommand(intake, 0.3);
  private final SetIntakeCommand stopIntake = new SetIntakeCommand(intake, 0);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // add the autos to the chooser
    AutoChooser.addAutos(doNothingAuto, driveOffLine, shootThenDrive, twoBallAuto, threeBallAuto);
    AutoChooser.addGamepad(gamepad1);

    // Configure the button bindings
    configureButtonBindings();
    //drivetrain.setDefaultCommand(teleopDrive); // this may work better than other scheduling hacks but might mess up auto idk
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      backButton.whenPressed(teleopDrive);

      aButton.whenPressed(intakeIn);
      aButton.whenReleased(stopIntake);
  }

  public Command getDriveCommand() {
    return teleopDrive;
  }
}
