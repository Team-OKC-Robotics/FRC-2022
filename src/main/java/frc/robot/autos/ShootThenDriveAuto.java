package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.SetSpeedDrive;
import frc.robot.commands.intake.SetIndexerCommand;
import frc.robot.commands.shooter.SetShooterCommand;
import frc.robot.commands.shooter.SetTriggerCommand;
import frc.robot.commands.shooter.ShootAndFeedCommand;
import frc.robot.commands.shooter.StopShooterCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Auto;

public class ShootThenDriveAuto extends Auto {
    public ShootThenDriveAuto(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IntakeSubsystem intake) {
        super(
            "Shoot then drive auto",
            "An auto that shoots the preloaded, then drives off the tarmac",
            6,
            new SetShooterCommand(shooter, 8500),
            new ShootAndFeedCommand(shooter, 8500, 0.4, 3),
            new StopShooterCommand(shooter),
            new SetIndexerCommand(intake, 0),
            new SetTriggerCommand(shooter, 0),
            // new DriveCommand(drivetrain, -70),
            // new DriveSetSpeedCommand(drivetrain, -20, -0.5),
            new SetSpeedDrive(drivetrain, -0.5, 3.5),
            new SetSpeedDrive(drivetrain, 0, 0.1),
            // new SetIntakePositionCommand(intatake, true), // deploy the intake to be ready for tele-op
            new WaitCommand(3) // wait for the intake to get fully deployed
        );
    }
}