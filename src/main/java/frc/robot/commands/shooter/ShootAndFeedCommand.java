package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.SetIndexerCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAndFeedCommand extends ParallelCommandGroup {
    public ShootAndFeedCommand(ShooterSubsystem shooter, IntakeSubsystem intake, double RPM, double seconds) {
        super(
            new SetShooterCommand(shooter, RPM),
            new SetIndexerCommand(intake, 1),
            new WaitCommand(seconds)
        );
    }
}
