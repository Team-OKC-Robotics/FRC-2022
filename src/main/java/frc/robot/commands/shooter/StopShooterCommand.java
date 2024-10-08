package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooterCommand extends CommandBase {
    private final ShooterSubsystem shooter;

    public StopShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.stopShooter();
        shooter.resetPower();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
