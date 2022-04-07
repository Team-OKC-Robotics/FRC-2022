package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAndFeedCommand extends WaitCommand {
    private final ShooterSubsystem shooter;
    private double RPM;
    private double power = 0.2;

    public ShootAndFeedCommand(ShooterSubsystem shooter, double RPM, double power, double seconds) {
        super(seconds);
        this.shooter = shooter;
        this.RPM = RPM;
        this.power = power;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooter(RPM);
        shooter.feed(power);
    }

    @Override // wait do I even need to override this? maybe not...
    public boolean isFinished() {
        return super.isFinished();
    }
}
