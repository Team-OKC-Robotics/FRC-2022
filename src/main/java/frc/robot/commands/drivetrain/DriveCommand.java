package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private double distance = 0;
    private double maxOutput = 1;

    /**
     * Drives the drivetrain on the heading it had when the command started, for the given distance
     * @param drivetrain the drivetrain subsystem for the command to control
     * @param distance the distance, in inches, to drive the robot
     */
    public DriveCommand(DrivetrainSubsystem drivetrain, double distance) {
        this.drivetrain = drivetrain;
        this.distance = distance;

        addRequirements(drivetrain);
    }

    /**
     * Drives the drivetrain on the heading it had when the command started, for the given distance
     * @param drivetrain the drivetrain subsystem for the command to control
     * @param distance the distance, in inches, to drive the robot
     */
    public DriveCommand(DrivetrainSubsystem drivetrain, double distance, double maxOutput) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        this.maxOutput = maxOutput;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetEncoders();
        drivetrain.resetDistancePID();
        drivetrain.resetHeadingPID();
        drivetrain.setHeading();
        drivetrain.setMaxOutput(maxOutput);
    }

    @Override
    public void execute() {
        drivetrain.driveDistance(distance);
    }

    @Override
    public void end(boolean executed) {
        // drivetrain.resetDistancePID();
        // drivetrain.resetEncoders();
        // drivetrain.resetGyro();
        // drivetrain.resetHeadingPID();
        drivetrain.setMaxOutput(1);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.atDistanceSetpoint() && drivetrain.atHeadingSetpoint();
    }
}
