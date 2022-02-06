package frc.robot.autos;

import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Auto;

public class DoNothingAuto extends Auto {
    public DoNothingAuto(DrivetrainSubsystem drivetrain) {
        super(
            "Do Nothing Auto",
            "An auto that does 'nothing,' in that it drives the robot forwards 0 inches",
            0,
            new DriveCommand(drivetrain, 0)
        );
    }
}