package frc.robot.autos;

import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Auto;

public class FindBALLSS extends Auto {
    public FindBALLSS(DrivetrainSubsystem drivetrain) {
        super(
            "",
            "",
            0,
            new DriveCommand(drivetrain, 50)
        );
    }
}
