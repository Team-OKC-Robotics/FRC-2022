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
                shooter.setShooterPreset(ShooterPresets.TARMAC_LINE);
                break;
            case 270:
                shooter.setShooterPreset(ShooterPresets.CLOSE_LAUNCHPAD);
                break;
            case 0:
            default:
                shooter.setShooterPreset(ShooterPresets.FAR_LAUNCHPAD);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return shooter.atShooterSetpoint();
    }
}
