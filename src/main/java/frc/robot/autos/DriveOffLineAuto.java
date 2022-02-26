package frc.robot.autos;

import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Auto;

public class DriveOffLineAuto extends Auto {
    public DriveOffLineAuto(DrivetrainSubsystem drivetrain) {
        super(
            "Drive Off Line Auto",
            "An auto that drives off the line forwards (away from alliance station), scoring points",
            2,
            new DriveCommand(drivetrain, 5)
        );
    }
}
