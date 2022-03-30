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
    public void initialize() {
        drivetrain.setOpenLoopRampRate(); // only get open loop ramp rate when we start tele op
        drivetrain.setSpeedModifier(0.75); // so as not to tip
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(Math.pow(-gamepad.getRawAxis(1), 3), Math.pow(gamepad.getRawAxis(4), 3), false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
