package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SetTriggerCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private double power;

    public SetTriggerCommand(ShooterSubsystem shooter, double power) {
        this.shooter = shooter;
        this.power = power;
        
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (power <= 0) {
            shooter.setTrigger(power);
        } else if (shooter.atShooterSetpoint()) {
            shooter.setTrigger(power);
        } else {
            shooter.setTrigger(0);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
