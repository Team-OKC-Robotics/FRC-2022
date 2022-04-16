package frc.robot.autos;

import frc.robot.commands.shooter.SetShooterCommand;
import frc.robot.commands.shooter.SetTriggerCommand;
import frc.robot.commands.shooter.ShootAndFeedCommand;
import frc.robot.commands.shooter.StopShooterCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Auto;

public class ShootButNoTaxiAuto extends Auto {
    public ShootButNoTaxiAuto(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter) {
        super(
            "Shoot but don't taxi auto",
            "An auto that shoots the preloaded, then doesn't do anything",
            6,
            new SetShooterCommand(shooter, 8500),
            new ShootAndFeedCommand(shooter, 8500, 0.4, 3),
            new StopShooterCommand(shooter),
            new SetTriggerCommand(shooter, 0)
        );
    }
}