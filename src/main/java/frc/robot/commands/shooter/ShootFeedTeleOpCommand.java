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
        if (shooter.atShooterSetpoint()) { // if shooter is at setpoint
            shooter.feed(power); // run the shooter
        } else {
            shooter.setTrigger(power); // has the ball detection so it lets the cargo go back up to where it's ready to shoot
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
