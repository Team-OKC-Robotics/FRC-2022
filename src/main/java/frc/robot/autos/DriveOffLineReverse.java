package frc.robot.autos;

import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Auto;

public class DriveOffLineReverse extends Auto {
    public DriveOffLineReverse(DrivetrainSubsystem drivetrain) {
        super(
            "Drive Off Line Reverse Auto",
            "An auto that drives off the line backwards (towards alliance station), scoring points",
            2,
            new DriveCommand(drivetrain, -50)
        );
    }
}