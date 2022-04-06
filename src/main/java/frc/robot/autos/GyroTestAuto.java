package frc.robot.autos;

import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.drivetrain.TurnCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Auto;

public class GyroTestAuto extends Auto {
    public GyroTestAuto(DrivetrainSubsystem drivetrain) {
        super(
            "GyroTestAuto",
            "an auto that tests the drivetrain so we can get all the kinks worked out",
            0,
            new DriveCommand(drivetrain, 10),
            new TurnCommand(drivetrain, 90),
            new DriveCommand(drivetrain, 10),
            new TurnCommand(drivetrain, 90),
            new DriveCommand(drivetrain, 10),
            new TurnCommand(drivetrain, 90),
            new DriveCommand(drivetrain, -10)
        );
    }
}