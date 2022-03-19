package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TeleOpDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final Joystick gamepad;

    public TeleOpDriveCommand(DrivetrainSubsystem drivetrain, Joystick gamepad) {
        this.drivetrain = drivetrain;
        this.gamepad = gamepad;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(Math.pow(-gamepad.getRawAxis(1), 3), Math.pow(gamepad.getRawAxis(4), 3), false);
        // drivetrain.curvatureDrive(-gamepad.getRawAxis(1), gamepad.getRawAxis(4), gamepad.getRawButton(1));
        // drivetrain.arcadeDrive(-gamepad.getRawAxis(1), gamepad.getRawAxis(2));

        // private final RunCommand teleopDrive = new RunCommand(() -> drivetrain.arcadeDrive(-gamepad1.getRawAxis(1), gamepad1.getRawAxis(4)), drivetrain);
        // private final RunCommand teleopDrive = new RunCommand(() -> drivetrain.tankDrive(-gamepad1.getRawAxis(1), -gamepad1.getRawAxis(5)), drivetrain);
        // private final RunCommand teleopDrive = new RunCommand(
        // () -> drivetrain.arcadeDrive(-gamepad1.getRawAxis(1), gamepad1.getRawAxis(2)), drivetrain);
  
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
