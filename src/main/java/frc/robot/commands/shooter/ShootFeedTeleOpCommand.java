package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootFeedTeleOpCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private double RPM;

    public ShootFeedTeleOpCommand(ShooterSubsystem shooter, IntakeSubsystem intake, double RPM) {
        this.shooter = shooter;
        this.intake = intake;
        this.RPM = RPM;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooter(RPM);
        if (shooter.atShooterSetpoint()) {
            intake.setIndexer(1);
        } else {
            intake.setIndexer(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIndexer(0);
        shooter.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return shooter.atShooterSetpoint();
    }
}
