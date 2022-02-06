package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShootK;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.drivetrain.TurnCommand;
import frc.robot.commands.intake.SetIntakeCommand;
import frc.robot.commands.shooter.SetShooterCommand;
import frc.robot.commands.shooter.SetTriggerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Auto;

public class TwoBallAuto extends Auto {
    public TwoBallAuto(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IntakeSubsystem intake) {
        super(
                "Two ball auto",
                "An auto that picks up the ball in front of it, turns around, then shoots it and the preloaded",
                6,
                new SetIntakeCommand(intake, 0.3),
                new DriveCommand(drivetrain, 30),
                new TurnCommand(drivetrain, 180),
                new SetShooterCommand(shooter, ShootK.tarmacPreset),
                new SetTriggerCommand(shooter, 1),
                new WaitCommand(10)
            );
    }
}
