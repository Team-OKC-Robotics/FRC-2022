// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.autos.*;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.RotateClimberCommand;
import frc.robot.commands.drivetrain.TeleOpDriveCommand;
import frc.robot.commands.intake.SetIntakeCommand;
import frc.robot.commands.shooter.SetShooterCommand;
import frc.robot.commands.vision.VisionAlignCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.AutoChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
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
  //private JoystickButton rightStickButton = new JoystickButton(gamepad1, 10);
  //private JoystickButton LeftStickButton = new JoystickButton(gamepad1, 9);

  private JoystickButton aButton2 = new JoystickButton(gamepad2, 1);
  private JoystickButton bButton2 = new JoystickButton(gamepad2, 2);
  private JoystickButton xButton2 = new JoystickButton(gamepad2, 3);
  private JoystickButton yButton2 = new JoystickButton(gamepad2, 4);
  private JoystickButton leftBumper2 = new JoystickButton(gamepad2, 5);
  private JoystickButton rightBumper2 = new JoystickButton(gamepad2, 6);
  private JoystickButton backButton2 = new JoystickButton(gamepad2, 7);
  private JoystickButton startButton2 = new JoystickButton(gamepad2, 8);
  private JoystickButton rightStickButton2 = new JoystickButton(gamepad2, 10);
  private JoystickButton LeftStickButton2 = new JoystickButton(gamepad2, 9);


  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();

  // autos
  private final DoNothingAuto doNothingAuto = new DoNothingAuto(drivetrain); // drives the robot 0 inches
  private final DriveOffLineAuto driveOffLine = new DriveOffLineAuto(drivetrain); // drives the robot forwards
  //private final ShootThenDriveAuto shootThenDrive = new ShootThenDriveAuto(drivetrain, shooter); // drives the robot backwards
  //private final TwoBallAuto twoBallAuto = new TwoBallAuto(drivetrain, shooter, intake); // drives the robot backwards
  //private final ThreeBallAuto threeBallAuto = new ThreeBallAuto(drivetrain, shooter, intake); // drives the robot backwards
  private final GyroTestAuto gyroTestAuto = new GyroTestAuto(drivetrain);

  // commands
  private final TeleOpDriveCommand teleOpDrive = new TeleOpDriveCommand(drivetrain, gamepad1);
  private final RunCommand teleopDrive = new RunCommand(() -> drivetrain.arcadeDrive(-gamepad1.getRawAxis(1), gamepad1.getRawAxis(4)), drivetrain);
  //private final RunCommand teleopDrive = new RunCommand(() -> drivetrain.tankDrive(-gamepad1.getRawAxis(1), -gamepad1.getRawAxis(5)), drivetrain);
//climber
private final ExtendClimberCommand Climber = new ExtendClimberCommand(climber,20, true);
private final ExtendClimberCommand Climber2 = new ExtendClimberCommand(climber,20, false);
private final ExtendClimberCommand Climber01 = new ExtendClimberCommand(climber,-20, true);
private final ExtendClimberCommand Climber02 = new ExtendClimberCommand(climber,-20, false);

private final RotateClimberCommand Climber001 = new RotateClimberCommand(climber,45,true);
private final RotateClimberCommand Climber002 = new RotateClimberCommand(climber,45,false);
  // intake
  private final SetIntakeCommand intakeIn = new SetIntakeCommand(intake, 0.3);
  private final SetIntakeCommand stopIntake = new SetIntakeCommand(intake, 0);

  // vision
  private final VisionAlignCommand visionAlign = new VisionAlignCommand(vision, drivetrain);

  // shooter
  private final SetShooterCommand slowShooter = new SetShooterCommand(shooter, 1000);
  private final SetShooterCommand fastShooter = new SetShooterCommand(shooter, 3000);
  private final SetShooterCommand maxShooter = new SetShooterCommand(shooter, 5000);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // add the autos to the chooser
    AutoChooser.addAutos(doNothingAuto, driveOffLine, /*shootThenDrive, twoBallAuto, threeBallAuto,*/ gyroTestAuto);
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
      backButton.whenPressed(teleOpDrive);
     

      leftBumper.whenPressed(intakeIn);
      leftBumper.whenReleased(stopIntake);

      yButton.whenPressed(visionAlign);

aButton.whenPressed(fastShooter);
bButton.whenPressed(slowShooter);
 xButton.whenPressed(maxShooter);
 startButton.whenPressed((Command) climber);

 private void configureButtonBindings() {
    backButton2.whenPressed(teleOpDrive);
    
    leftBumper2.whenPressed(Climber);
    leftBumper2.whenReleased(Climber01);
    rightBumper2.whenPressed(Climber2);
    rightBumper2.whenReleased(Climber02);

    LeftStickButton2.whenPressed(Climber001);
    //LeftStickButton2.whenReleased(Climber001);

    rightStickButton2.whenPressed(Climber002);

  }

  public Command getDriveCommand() {
    //return teleopDrive;
    return teleOpDrive;
  }
}
