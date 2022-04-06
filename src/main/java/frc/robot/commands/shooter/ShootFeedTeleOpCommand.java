package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootFeedTeleOpCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private double RPM;
    private double power = 0.4;

    public ShootFeedTeleOpCommand(ShooterSubsystem shooter, double RPM, double power) {
        this.shooter = shooter;
        this.RPM = RPM;
        this.power = power;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooter(RPM);
        shooter.feed(power); // run the shooter
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
