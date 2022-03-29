package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShootK;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.drivetrain.DriveSetSpeedCommand;
import frc.robot.commands.drivetrain.SetSpeedDrive;
import frc.robot.commands.drivetrain.TeleOpDriveCommand;
import frc.robot.commands.intake.SetIndexerCommand;
import frc.robot.commands.intake.SetIntakePositionCommand;
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
            new ParallelCommandGroup(new SetShooterCommand(shooter, 8000), new WaitCommand(2)), // warm up the shooter
            new ShootAndFeedCommand(shooter, ShootK.preset1, 3), // once it's there run both the shooter and the indexer for 3 seconds
            new StopShooterCommand(shooter),
            new SetIndexerCommand(intake, 0),
            new SetTriggerCommand(shooter, 0),
            // new DriveCommand(drivetrain, -70),
            // new DriveSetSpeedCommand(drivetrain, -30, 0.5),
            // new DriveSetSpeedCommand(drivetrain, -20, -0.5),
            // new ParallelCommandGroup(new SetSpeedDrive(drivetrain, -1), new WaitCommand(3)),
            new SetSpeedDrive(drivetrain, -0.5, 3.5),
            new SetSpeedDrive(drivetrain, 0, 0.1),
            // new SetIntakePositionCommand(intatake, true), // deploy the intake to be ready for tele-op
            new WaitCommand(3) // wait for the intake to get fully deployed
        );
    }
}