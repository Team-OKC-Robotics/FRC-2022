package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class FlightStickShooterCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private Joystick gamepad;

    public FlightStickShooterCommand(ShooterSubsystem shooter, Joystick gamepad) {
        this.shooter = shooter;
        this.gamepad = gamepad;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooter(gamepad.getRawAxis(3) * 10000);
    }

    @Override
    public boolean isFinished() {
        return shooter.atShooterSetpoint();
    }
}
