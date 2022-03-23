package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShootK;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.intake.SetIndexerCommand;
import frc.robot.commands.intake.SetIntakePositionCommand;
import frc.robot.commands.shooter.SetShooterCommand;
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
            new ParallelCommandGroup(new SetShooterCommand(shooter, 8000), new WaitCommand(1.5)), // warm up the shooter
            new ShootAndFeedCommand(shooter, intake, ShootK.preset1, 3), // once it's there run both the shooter and the indexer for 3 seconds
            new StopShooterCommand(shooter),
            new SetIndexerCommand(intake, 0),
            new DriveCommand(drivetrain, -80),
            new SetIntakePositionCommand(intake, true), // deploy the intake to be ready for tele-op
            new WaitCommand(3) // wait for the intake to get fully deployed
        );
    }
}
//Danial, if you read this, run
//to where? and you spelt my name wrong like what the heck