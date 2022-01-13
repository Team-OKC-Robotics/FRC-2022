package autos;

import frc.robot.util.Auto;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveOffLineReverse extends Auto {
    public DriveOffLineReverse(DrivetrainSubsystem drivetrain) {
        super(
            "Drive Off Line Reverse Auto",
            "An auto that drives off the line backwards (towards alliance station), scoring points",
            -1,
            new DriveCommand(drivetrain, -50)
        );
    }
}