package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class QuickTeleOpDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final Joystick gamepad;

    public QuickTeleOpDriveCommand(DrivetrainSubsystem drivetrain, Joystick gamepad) {
        this.drivetrain = drivetrain;
        this.gamepad = gamepad;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setOpenLoopRampRate(0.1); // quicker acceleration
        drivetrain.setSpeedModifier(1); // literally max speed (be careful to not tip though)
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
