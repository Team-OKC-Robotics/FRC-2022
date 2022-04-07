package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class TimedShooterCommand extends WaitCommand {
    private final ShooterSubsystem shooter;
    private double RPM;

    public TimedShooterCommand(ShooterSubsystem shooter, double RPM, double seconds) {
        super(seconds);
        this.shooter = shooter;
        this.RPM = RPM;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooter(RPM);
    }

    @Override // wait do I even need to override this? maybe not...
    public boolean isFinished() {
        return super.isFinished();
    }
}
