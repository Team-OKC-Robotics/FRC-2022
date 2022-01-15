package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private double RPM;

    public SetShooterCommand(ShooterSubsystem shooter, double RPM) {
        this.shooter = shooter;
        this.RPM = RPM;
    }

    @Override
    public void execute() {
        shooter.set(RPM);
    }

    @Override
    public boolean isFinished() {
        return shooter.atShooterSetpoint();
    }
}
