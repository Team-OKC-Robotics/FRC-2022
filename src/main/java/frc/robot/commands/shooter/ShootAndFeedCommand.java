package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAndFeedCommand extends ParallelCommandGroup {
    public ShootAndFeedCommand(ShooterSubsystem shooter, double RPM, double seconds) {
        super(
            new SetShooterCommand(shooter, RPM),
            new FeedCommand(shooter, 1),
            new WaitCommand(seconds)
        );
    }
}
