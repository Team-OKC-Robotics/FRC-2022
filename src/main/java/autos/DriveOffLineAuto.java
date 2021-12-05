package autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveOffLineAuto extends SequentialCommandGroup {
    public DriveOffLineAuto(DrivetrainSubsystem drivetrain) {
        super(
            new DriveCommand(drivetrain, 10)
        );
    }
}
