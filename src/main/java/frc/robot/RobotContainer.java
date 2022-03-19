// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.autos.*;
import frc.robot.commands.climber.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.vision.*;
import frc.robot.subsystems.*;
import frc.robot.util.AutoChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // gamepads
  private final Joystick gamepad1 = new Joystick(0);
  private final Joystick gamepad2 = new Joystick(1);
  private final Joystick gamepad3 = new Joystick(2);

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

  // private JoystickButton aButton2 = new JoystickButton(gamepad2, 1);
  // private JoystickButton bButton2 = new JoystickButton(gamepad2, 2);
  // private JoystickButton xButton2 = new JoystickButton(gamepad2, 3);
  // private JoystickButton yButton2 = new JoystickButton(gamepad2, 4);
  // private JoystickButton leftBumper2 = new JoystickButton(gamepad2, 5);
  // private JoystickButton rightBumper2 = new JoystickButton(gamepad2, 6);
  // private JoystickButton backButton2 = new JoystickButton(gamepad2, 7);
  // private JoystickButton startButton2 = new JoystickButton(gamepad2, 8);
  // private JoystickButton rightStickButton2 = new JoystickButton(gamepad2, 10);
  // private JoystickButton leftStickButton2 = new JoystickButton(gamepad2, 9);

  private JoystickButton triggerButton = new JoystickButton(gamepad3, 1); // 1
  private JoystickButton sideButton = new JoystickButton(gamepad3, 2); // 2
  private JoystickButton topLeftButton = new JoystickButton(gamepad3, 5); // 5 
  private JoystickButton topRightButton = new JoystickButton(gamepad3, 6); // 6
  private JoystickButton bottomLeftButton = new JoystickButton(gamepad3, 3); // 3
  private JoystickButton bottomRightButton = new JoystickButton(gamepad3, 4); // 4
  private JoystickButton sevenButton = new JoystickButton(gamepad3, 7); // as they are named
  private JoystickButton eightButton = new JoystickButton(gamepad3, 8); // as they are named
  private JoystickButton nineButton = new JoystickButton(gamepad3, 9); // as they are named
  private JoystickButton tenButton = new JoystickButton(gamepad3, 10); // as they are named
  private JoystickButton elevenButton = new JoystickButton(gamepad3, 11); // as they are named
  private JoystickButton twelveButton = new JoystickButton(gamepad3, 12); // 12
  

  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();

  // autos
  private final DoNothingAuto doNothingAuto = new DoNothingAuto(drivetrain); // drives the robot 0 inches
  private final DriveOffLineAuto driveOffLine = new DriveOffLineAuto(drivetrain); // drives the robot forwards
  private final ShootThenDriveAuto shootThenDrive = new ShootThenDriveAuto(drivetrain, shooter, intake); // drives the robot backwards
  private final TwoBallAuto twoBallAuto = new TwoBallAuto(drivetrain, shooter, intake); // drives the robot backwards
  private final ThreeBallAuto threeBallAuto = new ThreeBallAuto(drivetrain, shooter, intake); // drives the robot backwards
  private final GyroTestAuto gyroTestAuto = new GyroTestAuto(drivetrain);

  // commands
  private final TeleOpDriveCommand teleOpDrive = new TeleOpDriveCommand(drivetrain, gamepad1);
  private final SlowTeleOpDrive slowTeleOpDrive = new SlowTeleOpDrive(drivetrain, gamepad1);

  // intake
  private final SetIntakeCommand intakeIn = new SetIntakeCommand(intake, 1);
  private final SetIntakeCommand stopIntake = new SetIntakeCommand(intake, 0);
  private final SetIntakeCommand intakeOut = new SetIntakeCommand(intake, -1);
  // private final SetIntakePositionCommand deployIntake = new SetIntakePositionCommand(intake, true);
  // private final SetIntakePositionCommand retractIntake = new SetIntakePositionCommand(intake, false);
  private final SetIntakePositionPOVCommand intakePositionPOVCommand = new SetIntakePositionPOVCommand(intake, gamepad3);
  private final SetIndexerCommand indexerIn = new SetIndexerCommand(intake, 0.2);
  private final FeedCommand feed = new FeedCommand(intake, 0.2);
  private final FeedCommand fastFeed = new FeedCommand(intake, 0.6);
  private final SetIndexerCommand indexerOut = new SetIndexerCommand(intake, -0.2);
  private final SetIndexerCommand stopIndexer = new SetIndexerCommand(intake, 0);

  // vision
  private final VisionAlignCommand visionAlign = new VisionAlignCommand(vision, drivetrain);
  private final SetLEDsCommand ledsOn = new SetLEDsCommand(vision, true);
  private final SetLEDsCommand ledsOff = new SetLEDsCommand(vision, false);

  // shooter
  private final StopShooterCommand stopShooter = new StopShooterCommand(shooter);
  // private final ShootFeedTeleOpCommand fastShooter = new ShootFeedTeleOpCommand(shooter, intake, 9500);
  private final SetShooterCommand fastShooter = new SetShooterCommand(shooter, 9000);
  // private final SetShooterCommand fastShooter = new SetShooterCommand(shooter, 5000);
  private final FlightStickShooterCommand flightStickShooter = new FlightStickShooterCommand(shooter, gamepad2); // expects gamepad2 to be a Logitech Extreme 3D Pro

  // climber
  private final ManualClimberCommand extendLeftClimber = new ManualClimberCommand(climber, 0.7, true);
  private final ManualClimberCommand retractLeftClimber = new ManualClimberCommand(climber, -0.7, true);
  private final ManualClimberCommand stopLeftClimber = new ManualClimberCommand(climber, 0, true);
  private final ManualClimberCommand extendRightClimber = new ManualClimberCommand(climber, 0.7, false);
  private final ManualClimberCommand retractRightClimber = new ManualClimberCommand(climber, -0.7, false);
  private final ManualClimberCommand stopRightClimber = new ManualClimberCommand(climber, 0, false);

  private final ManualRotateClimberCommand rotateLeftClimber = new ManualRotateClimberCommand(climber, gamepad3, true);
  private final ManualStopRotateClimberCommand stopLeftRotate = new ManualStopRotateClimberCommand(climber, true);
  private final ManualRotateClimberCommand rotateRightClimber = new ManualRotateClimberCommand(climber, gamepad3, false);
  private final ManualStopRotateClimberCommand stopRightRotate = new ManualStopRotateClimberCommand(climber, false);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // add the autos to the chooser
    AutoChooser.addAutos(doNothingAuto, driveOffLine, shootThenDrive, twoBallAuto, threeBallAuto, gyroTestAuto);
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
      aButton.whenPressed(slowTeleOpDrive).whenReleased(teleOpDrive); // for lining up climber and stuff
      // aButton.whenPressed(visionAlign).whenReleased(teleOpDrive);

      // intake commands
      leftBumper.whenPressed(intakeIn).whenReleased(stopIntake);
      rightBumper.whenPressed(intakeOut).whenReleased(stopIntake);

      // fun flight stick controls (this is really fun can confirm)
      // shooter
      sideButton.whenPressed(fastShooter).whenReleased(stopShooter);
      
      // climber
      topLeftButton.whenPressed(extendLeftClimber).whenReleased(stopLeftClimber);
      bottomLeftButton.whenPressed(retractLeftClimber).whenReleased(stopLeftClimber);
      topLeftButton.whenPressed(extendRightClimber).whenReleased(stopRightClimber);
      bottomLeftButton.whenPressed(retractRightClimber).whenReleased(stopRightClimber);
      twelveButton.whenPressed(rotateLeftClimber).whenReleased(stopLeftRotate);
      twelveButton.whenPressed(rotateRightClimber).whenReleased(stopRightRotate);
      
      // intake
      triggerButton.whenPressed(feed).whenReleased(stopIndexer);
      sevenButton.whenPressed(indexerOut).whenReleased(stopIndexer);
      eightButton.whileHeld(indexerIn).whenReleased(stopIndexer);
      nineButton.whenPressed(fastFeed).whenReleased(stopIndexer);
  }

  public Command getDriveCommand() {
    // return teleopDrive;
    return teleOpDrive;
  }

  public Command getIntakePositionCommand() {
    return intakePositionPOVCommand;
  }
}
