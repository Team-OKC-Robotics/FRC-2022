package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private double distance = 0;

    public DriveCommand(DrivetrainSubsystem drivetrain, double distance) {
        this.drivetrain = drivetrain;
        this.distance = distance;
    }

    public void execute() {
        drivetrain.driveDistance(distance);
    }

    public boolean isFinished(boolean interrupted) {
        return drivetrain.atDistanceSetpoint();
    }
}
