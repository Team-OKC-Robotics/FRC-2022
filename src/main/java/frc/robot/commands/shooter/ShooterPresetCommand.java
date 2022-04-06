package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterPresets;

public class ShooterPresetCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private Joystick gamepad;
    private double power;
    private ShooterPresets preset;

    public ShooterPresetCommand(ShooterSubsystem shooter, Joystick gamepad, double power) {
        this.shooter = shooter;
        this.gamepad = gamepad;
        this.power = power;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        switch(gamepad.getPOV()) {
            case 180:
                preset = ShooterPresets.AGAINST_HUB;
                break;
            case 0:
                preset = ShooterPresets.NORMAL_SHOT;
                break;
        }
        if (preset != null) {
            shooter.setShooterPreset(preset);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false; // just run until cancelled by the scheduling of StopShooterCommand
    }
}