package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShootK;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.drivetrain.DriveSetSpeedCommand;
import frc.robot.commands.drivetrain.TurnCommand;
import frc.robot.commands.intake.SetIndexerCommand;
import frc.robot.commands.intake.SetIntakeCommand;
import frc.robot.commands.intake.SetIntakePositionCommand;
import frc.robot.commands.shooter.SetShooterCommand;
import frc.robot.commands.shooter.ShootAndFeedCommand;
import frc.robot.commands.shooter.StopShooterCommand;
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
                // new SetIntakePositionCommand(intake, true),
                // new WaitCommand(0.5), // for the intake to deploy
                // new SetIntakeCommand(intake, 1),
                // new DriveCommand(drivetrain, 40, 0.45), // max output of 0.45
                // new WaitCommand(5), // wait for the balls to make their way to the indexer
                // new SetIndexerCommand(intake, 1), // speed them up through
                // new WaitCommand(1),
                // new SetIndexerCommand(intake, 0), // and stop them
                // new TurnCommand(drivetrain, 180), // now with ball safely in we can turn
                // new DriveCommand(drivetrain, 40), // drive back to where we were
                // new SetShooterCommand(shooter, ShootK.tarmacPreset), // warm the shooter up
                // new SetTriggerCommand(intake, 1), // and shoot
                // new WaitCommand(10)
                
                // first shot
                new SetIntakePositionCommand(intake, false), // keep the intake up through the fast turning
                new ParallelCommandGroup(new SetShooterCommand(shooter, 8000), new WaitCommand(2)), // warm up the shooter
                new ShootAndFeedCommand(shooter, intake, ShootK.preset1, 1), // once it's there run both the shooter and the indexer for 3 seconds
                new StopShooterCommand(shooter), // stop the shooter
                new SetIndexerCommand(intake, 0), // stop the intake
                // new WaitCommand(0.5), // wait for the intake to get fully deployed
                
                // next shot
                new DriveCommand(drivetrain, -30),
                new TurnCommand(drivetrain, 180, 0.7), // turn towards the ball
                new SetIntakePositionCommand(intake, true), // deploy the intake to be ready for picking up balls
                new SetIntakeCommand(intake, 1), // intake
                new DriveSetSpeedCommand(drivetrain, 40, 0.3), // drive to pick up the balls (slowly so we don't run into anything too hard)
                // new SetIntakePositionCommand(intake, false), // raise the intake back up so if something goes wrong we don't wreck it
                new TurnCommand(drivetrain, -180), // turn back
                new DriveCommand(drivetrain, 70), // drive back
                new ParallelCommandGroup(new SetShooterCommand(shooter, 8000), new WaitCommand(1)), // warm up the shooter
                new ShootAndFeedCommand(shooter, intake, ShootK.preset1, 3), // once it's there run both the shooter and the indexer for 3 seconds
                new StopShooterCommand(shooter), // stop the shooter
                new SetIndexerCommand(intake, 0), // stop the intake
                new SetIndexerCommand(intake, 0) // stop the indexer
            );
    }
}
