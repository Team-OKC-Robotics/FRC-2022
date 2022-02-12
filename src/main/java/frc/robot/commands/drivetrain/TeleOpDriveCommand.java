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
        //drivetrain.arcadeDrive(-gamepad.getRawAxis(1), gamepad.getRawAxis(4));
        drivetrain.arcadeDrive(-gamepad.getRawAxis(1), gamepad.getRawAxis(2));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
