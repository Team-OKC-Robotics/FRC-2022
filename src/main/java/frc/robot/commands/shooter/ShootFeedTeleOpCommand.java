package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootFeedTeleOpCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private double RPM;

    public ShootFeedTeleOpCommand(ShooterSubsystem shooter, double RPM) {
        this.shooter = shooter;
        this.RPM = RPM;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooter(RPM);
        if (shooter.atShooterSetpoint()) {
            shooter.feed(1);
        } else {
            shooter.feed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.feed(0);
        shooter.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
