package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShootK;
import frc.robot.commands.drivetrain.DriveSetSpeedCommand;
import frc.robot.commands.drivetrain.TurnCommand;
import frc.robot.commands.intake.SetIndexerCommand;
import frc.robot.commands.intake.SetIntakeCommand;
import frc.robot.commands.intake.SetIntakePositionCommand;
import frc.robot.commands.shooter.SetShooterCommand;
import frc.robot.commands.shooter.ShootAndFeedCommand;
import frc.robot.commands.shooter.StopShooterCommand;
import frc.robot.commands.vision.VisionAlignCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Auto;

public class TwoBallAuto extends Auto {
    public TwoBallAuto(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem vision) {
        super(
                "Two ball auto",
                "An auto that shoots the preloaded, turns around, picks up the ball behind it, goes back, and shoots it",
                6,
                new SetIntakePositionCommand(intake, true), // deploy the intake
                new SetIntakeCommand(intake, 1), // run the intake
                new SetIndexerCommand(intake, 0.4), // and indexer
                new WaitCommand(1), // wait for intake to reach fully deployed

                new DriveSetSpeedCommand(drivetrain, 40, 0.3), // drive to pick up the balls (slowly so we don't run into anything too hard)
                new TurnCommand(drivetrain, -180), // turn back
                new VisionAlignCommand(vision, drivetrain),
                new SetShooterCommand(shooter, 8500), // warmup the shooter
                new DriveSetSpeedCommand(drivetrain, 40, 0.4), // drive back

                new ShootAndFeedCommand(shooter, ShootK.normalShot, 0.4, 6), // once it's there run both the shooter and the indexer for 6 seconds
                new StopShooterCommand(shooter), // stop the shoote
                new SetIntakeCommand(intake, 0), // stop the intake
                new SetIndexerCommand(intake, 0) // stop the indexer
            );
    }
}
