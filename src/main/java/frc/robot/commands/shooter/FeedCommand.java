package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private double power;

    public FeedCommand(ShooterSubsystem shooter, double power) {
        this.shooter = shooter;
        this.power = power;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.feed(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
