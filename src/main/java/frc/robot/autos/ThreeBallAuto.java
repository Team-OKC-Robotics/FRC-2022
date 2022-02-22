package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShootK;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.drivetrain.TurnCommand;
import frc.robot.commands.intake.SetIntakeCommand;
import frc.robot.commands.shooter.SetShooterCommand;
import frc.robot.commands.shooter.SetTriggerCommand;
import frc.robot.commands.shooter.StopShooterCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Auto;

public class ThreeBallAuto extends Auto {
    public ThreeBallAuto(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IntakeSubsystem intake) {
        super(
                "Three ball auto",
                "Shoots preloaded, picks up closest two same-alliance balls, then shoots them",
                8,
                new SetShooterCommand(shooter, ShootK.tarmacPreset),
                new SetTriggerCommand(intake, 1),
                new WaitCommand(4),
                new StopShooterCommand(shooter),
                new SetTriggerCommand(intake, 0),
                new TurnCommand(drivetrain, 180),
                new SetIntakeCommand(intake, 0.3),
                new DriveCommand(drivetrain, 30),
                new TurnCommand(drivetrain, 90),
                new DriveCommand(drivetrain, 60),
                new TurnCommand(drivetrain, -30),
                new SetShooterCommand(shooter, ShootK.tarmacPreset),
                new SetTriggerCommand(intake, 1),
                new WaitCommand(10)
            );
    }
}
