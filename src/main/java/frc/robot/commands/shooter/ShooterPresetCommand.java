package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterPresets;

public class ShooterPresetCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private Joystick gamepad;

    public ShooterPresetCommand(ShooterSubsystem shooter, Joystick gamepad) {
        this.shooter = shooter;
        this.gamepad = gamepad;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        switch(gamepad.getPOV()) {
            case 90:
                shooter.setShooterPreset(ShooterPresets.LOW_GOAL);
                break;
            case 180:
                shooter.setShooterPreset(ShooterPresets.AGAINST_HUB);
                break;
            case 270:
                shooter.setShooterPreset(ShooterPresets.NORMAL_SHOT);
                break;
            case 0:
                shooter.setShooterPreset(ShooterPresets.FAR_SHOT);
                break;
            default:
                shooter.setShooterPreset(ShooterPresets.NORMAL_SHOT);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return shooter.atShooterSetpoint();
    }
}