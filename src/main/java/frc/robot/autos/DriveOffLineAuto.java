package autos;

import frc.robot.util.Auto;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveOffLineAuto extends Auto {
    public DriveOffLineAuto(DrivetrainSubsystem drivetrain) {
        super(
            "Drive Off Line Auto",
            "An auto that drives off the line forwards (away from alliance station), scoring points",
            -1,
            new DriveCommand(drivetrain, 50)
        );
    }
}
