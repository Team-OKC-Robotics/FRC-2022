package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooter.SetShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Auto;

public class SysIdAuto extends Auto {
    public SysIdAuto(ShooterSubsystem shooter) {
        super(
            "no",
            "no",
            0,
            new SetShooterCommand(shooter, 0.05),
            new WaitCommand(2),
            new SetShooterCommand(shooter, 0.1),
            new WaitCommand(2),
            new SetShooterCommand(shooter, 0.15),
            new WaitCommand(2),
            new SetShooterCommand(shooter, 0.20),
            new WaitCommand(2),
            new SetShooterCommand(shooter, 0.25),
            new WaitCommand(2),
            new SetShooterCommand(shooter, 0.3),
            new WaitCommand(2),
            new SetShooterCommand(shooter, 0.35),
            new WaitCommand(2),
            new SetShooterCommand(shooter, 0.40),
            new WaitCommand(2),
            new SetShooterCommand(shooter, 0.45),
            new WaitCommand(2),
            new SetShooterCommand(shooter, 0.5),
            new WaitCommand(2),
            new SetShooterCommand(shooter, 0.55),
            new WaitCommand(2),
            new SetShooterCommand(shooter, 0.60),
            new WaitCommand(2),
            new SetShooterCommand(shooter, 0.65),
            new WaitCommand(2),
            new SetShooterCommand(shooter, 0.7),
            new WaitCommand(2),
            new SetShooterCommand(shooter, 0.75),
            new WaitCommand(2),
            new SetShooterCommand(shooter, 0.8),
            new WaitCommand(2)
        );
    }
}